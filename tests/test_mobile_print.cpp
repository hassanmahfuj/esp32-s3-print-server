#include <cstring>
#include <cassert>
#include <iostream>
#include <fstream>
#include "MobilePrint.h"
using namespace mobileprint;
std::vector<uint8_t> printed;
bool usbReady=true, usbOk=true, flushOk=true, active=false;
bool writer(const uint8_t *p,size_t n) { if(!usbOk)return false;printed.insert(printed.end(),p,p+n);return true; }
bool ready(){return usbReady;}
bool flush(){return flushOk;}
void activity(bool v){active=v;}
void put32(std::vector<uint8_t> &v,size_t p,uint32_t n){for(int i=3;i>=0;--i){v[p+i]=n;n>>=8;}}
std::vector<uint8_t> page(uint32_t depth=1,uint32_t color=18,uint32_t dpi=300,bool duplex=false,bool tumble=false) {
  std::vector<uint8_t> p(1796);
  put32(p,272,duplex?1:0);put32(p,276,dpi);put32(p,280,dpi);put32(p,340,1);put32(p,368,tumble?1:0);
  const uint32_t width=dpi==600?4960:2480,height=dpi==600?7016:3508;
  put32(p,372,width);put32(p,376,height);put32(p,384,depth);put32(p,388,depth);
  const uint32_t count=(width*depth+7)/8;
  put32(p,392,count);put32(p,400,color);put32(p,420,1);
  for(unsigned y=0;y<height;) {
    unsigned repeat=std::min(256u,height-y);p.push_back(repeat-1);
    unsigned left=count;
    while(left){unsigned run=std::min(128u,left);p.push_back(run-1);p.push_back(color==3?0:255);left-=run;}
    y+=repeat;
  }
  return p;
}
std::vector<uint8_t> raster(uint32_t depth=1,uint32_t color=18,uint32_t dpi=300,bool duplex=false,bool tumble=false){std::vector<uint8_t> v={'R','a','S','2'};auto p=page(depth,color,dpi,duplex,tumble);v.insert(v.end(),p.begin(),p.end());return v;}
void decode(const std::vector<uint8_t>&v,PwgRaster&d){for(uint8_t b:v){while(d.pending())assert(d.drain());assert(d.feed(b));}while(d.pending())assert(d.drain());}
Response request(uint16_t op,uint32_t jobId=0){Request q;q.id=42;Response r(q,op);r.text(0x45,"printer-uri","ipp://192.168.4.1:631/ipp/print");if(jobId)r.integer("job-id",jobId);return r;}
std::shared_ptr<Socket> send(const std::vector<uint8_t>&body,bool chunked=false,bool expect=false) {
  auto p=std::make_shared<Socket>();
  std::string h="POST /ipp/print HTTP/1.1\r\nContent-Type: application/ipp\r\n";
  h+=chunked?"Transfer-Encoding: chunked\r\n":"Content-Length: "+std::to_string(body.size())+"\r\n";
  if(expect) h+="Expect: 100-continue\r\n";
  h+="\r\n";
  p->input.assign(h.begin(),h.end());
  if(chunked){for(size_t pos=0;pos<body.size();){size_t n=std::min(size_t(113),body.size()-pos);char b[32];int l=snprintf(b,sizeof(b),"%zx\r\n",n);p->input.insert(p->input.end(),b,b+l);p->input.insert(p->input.end(),body.begin()+pos,body.begin()+pos+n);p->input.push_back('\r');p->input.push_back('\n');pos+=n;}const char *end="0\r\n\r\n";p->input.insert(p->input.end(),end,end+strlen(end));}
  else p->input.insert(p->input.end(),body.begin(),body.end());
  WiFiServer::incoming.push_back(p);return p;
}
void run(Server &s,const std::shared_ptr<Socket>&p){for(unsigned i=0;i<30000&&p->open;++i){s.poll();++testMillis;}assert(!p->open);}
uint16_t responseStatus(const Socket&p){std::string s(p.output.begin(),p.output.end());size_t pos=s.find("HTTP/1.1 200 OK");assert(pos!=s.npos);pos=s.find("\r\n\r\n",pos)+4;assert(pos+8<=s.size());return be16(p.output.data()+pos+2);}
// sRGB 8-bit A4 page at 300 dpi; pixel colour depends on (x, y) so cropping errors show.
uint8_t shade(uint32_t x,uint32_t y,int c){return c==0?(x/40)*7:c==1?(y/50)*3:((x+y)%5==0)*200;}
std::vector<uint8_t> rgbPage(){
  std::vector<uint8_t> p(1796);const uint32_t w=2480,h=3508;
  put32(p,276,300);put32(p,280,300);put32(p,372,w);put32(p,376,h);put32(p,384,8);put32(p,388,24);
  put32(p,392,w*3);put32(p,400,19);put32(p,420,1);
  for(uint32_t y=0;y<h;++y){
    p.push_back(0);
    // Alternate literal and repeat runs, counted in pixels.
    for(uint32_t x=0;x<w;){
      uint32_t n=std::min(100u,w-x);
      if((x/100)%2){p.push_back(257-n);for(uint32_t i=0;i<n;++i)for(int c=0;c<3;++c)p.push_back(shade(x+i,y,c));}
      else{bool flat=true;for(uint32_t i=1;i<n&&flat;++i)for(int c=0;c<3;++c)flat&=shade(x+i,y,c)==shade(x,y,c);
        if(flat){p.push_back(n-1);for(int c=0;c<3;++c)p.push_back(shade(x,y,c));}
        else{p.push_back(257-n);for(uint32_t i=0;i<n;++i)for(int c=0;c<3;++c)p.push_back(shade(x+i,y,c));}}
      x+=n;
    }
  }
  return p;
}
uint32_t le32(const uint8_t *p){return p[0]|p[1]<<8|p[2]<<16|uint32_t(p[3])<<24;}
// Walks an ESC/P-R job and checks every raster line against the source image.
void checkEscpr(unsigned pages){
  std::string out(printed.begin(),printed.end());
  size_t pos=out.find("ESCPR");assert(pos!=out.npos);pos+=5;
  assert(out.compare(0,4,std::string("\0\0\0\x1b",4))==0);
  std::vector<std::string> names;unsigned lines=0,seenPages=0;
  while(pos<out.size()&&printed[pos]==0x1b&&pos+10<=out.size()&&out[pos+1]!='@'){
    uint32_t n=le32(&printed[pos+2]);std::string name=out.substr(pos+6,4);const uint8_t *q=&printed[pos+10];
    if(name=="dsnd"){
      uint32_t x=be16(q),y=be16(q+2),size=be16(q+5);assert(x==0&&y<3437&&size+7==n);
      std::vector<uint8_t> row;
      if(q[4]==0)row.assign(q+7,q+7+size);
      else for(const uint8_t *r=q+7;r<q+7+size;){uint8_t c=*r++;if(c>=128){for(int i=0;i<257-c;++i)row.insert(row.end(),r,r+3);r+=3;}else{row.insert(row.end(),r,r+3*(c+1));r+=3*(c+1);}}
      assert(row.size()==2410*3);
      for(uint32_t i=0;i<2410;++i)for(int c=0;c<3;++c)assert(row[i*3+c]==shade(i+35,y+35,c));
      ++lines;
    } else {
      names.push_back(name);
      if(name=="endp"){++seenPages;assert(n==1&&q[0]==(seenPages<pages?1:0));}
      if(name=="setj"){assert(n==22&&be32(q)==2480&&be32(q+4)==3507&&be16(q+8)==35&&be32(q+12)==2410&&be32(q+16)==3437&&q[20]==2);}
      if(name=="setq"){assert(n==9&&q[2]==0);}
    }
    pos+=10+n;
  }
  assert(names.size()==4+2*pages&&names[0]=="setq"&&names[1]=="seti"&&names[2]=="setj"&&names.back()=="endj");
  assert(seenPages==pages&&lines==3437*pages);
  assert(out.compare(pos,2,"\x1b@")==0&&out.find("JE",pos)!=out.npos);
}
std::vector<uint8_t> printBody(){auto r=request(PrintJob);r.end();auto v=raster();r.bytes.insert(r.bytes.end(),v.begin(),v.end());return r.bytes;}
int main(int argc,char **argv){
  PwgRaster d(writer);decode(raster(),d);assert(d.finish());assert(d.pages()==1);
  std::string pcl(printed.begin(),printed.end());assert(pcl.find("@PJL ENTER LANGUAGE=PCL")!=pcl.npos);assert(pcl.find("\033*r2330S")!=pcl.npos);
  // Every emitted row of an all-white image must remain white after polarity conversion.
  size_t pos=0,rows=0;while((pos=pcl.find("\033*b292W",pos))!=pcl.npos){pos+=7;for(size_t k=0;k<292;++k)assert(printed[pos+k]==0);pos+=292;++rows;}assert(rows==3358);
  printed.clear();d.reset();decode(raster(1,3),d);assert(d.finish());
  d.reset();decode(raster(8,18),d);assert(d.finish());
  // DeviceK all-black must stay black, with unused trailing PCL bits cleared.
  printed.clear();d.reset();auto black=raster(1,3);
  for(size_t i=4+1796;i<black.size();){++i;for(int run=0;run<3;++run){++i;black[i++]=255;}}
  decode(black,d);assert(d.finish());pcl.assign(printed.begin(),printed.end());pos=pcl.find("\033*b292W")+7;
  for(size_t k=0;k<291;++k) assert(printed[pos+k]==255);
  assert(printed[pos+291]==0xc0);
  // 600 dpi uses genuine double-sized input, doubled crop and matching PCL units.
  printed.clear();d.reset();decode(raster(8,18,600),d);assert(d.finish());
  pcl.assign(printed.begin(),printed.end());
  assert(pcl.find("\033&u600D\033*p0X\033*p150Y\033*t600R")!=pcl.npos);
  assert(pcl.find("\033*r4660S\033*r6716T")!=pcl.npos);
  pos=0;rows=0;while((pos=pcl.find("\033*b583W",pos))!=pcl.npos){pos+=7;for(size_t k=0;k<583;++k)assert(printed[pos+k]==0);pos+=583;++rows;}assert(rows==6716);
  d.reset();decode(raster(1,3,600),d);assert(d.finish());
  d.reset();decode(raster(1,18,600),d);assert(d.finish());
  // PCL duplex modes for HP P2015d: long-edge (1) and short-edge (2).
  printed.clear(); d.reset(); d.setDuplex("two-sided-long-edge"); decode(raster(),d); assert(d.finish());
  pcl.assign(printed.begin(),printed.end()); assert(pcl.find("\033&l1S")!=pcl.npos);
  printed.clear(); d.reset(); d.setDuplex("two-sided-short-edge"); decode(raster(),d); assert(d.finish());
  pcl.assign(printed.begin(),printed.end()); assert(pcl.find("\033&l2S")!=pcl.npos);
  printed.clear(); d.reset(); d.setDuplex("two-sided-long-edge"); decode(raster(1,18,300,true),d); assert(d.finish());
  d.reset();auto multi=raster();auto second=page();multi.insert(multi.end(),second.begin(),second.end());decode(multi,d);assert(d.finish()&&d.pages()==2);
  d.reset();auto truncated=raster();truncated.pop_back();for(uint8_t b:truncated){while(d.pending())assert(d.drain());assert(d.feed(b));}assert(!d.finish());
  d.reset();auto wrong=raster();put32(wrong,4+276,1200);bool rejected=false;for(uint8_t b:wrong){if(!d.feed(b)){rejected=true;break;}}assert(rejected);
  d.reset();auto badRun=raster();badRun[4+1796+1]=128;rejected=false;for(uint8_t b:badRun){if(!d.feed(b)){rejected=true;break;}}assert(rejected);
  d.reset();usbOk=false;rejected=false;for(uint8_t b:raster()){if(!d.feed(b)){rejected=true;break;}}assert(rejected);usbOk=true;
  auto r=request(ValidateJob);r.integer("copies",2);r.end();RequestParser parser;int result=0;for(uint8_t b:r.bytes)result=parser.feed(b);assert(result==1&&parser.request().status==AttributesUnsupported);
  r=request(ValidateJob);r.text(0x44,"sides","two-sided-long-edge");r.end();parser.reset();for(uint8_t b:r.bytes)result=parser.feed(b);assert(result==1&&parser.request().status==Ok&&parser.request().sides=="two-sided-long-edge");
  r=request(ValidateJob);r.text(0x49,"document-format","application/pdf");r.end();parser.reset();for(uint8_t b:r.bytes)result=parser.feed(b);assert(result==1&&parser.request().status==FormatUnsupported);
  // End-of-attributes byte embedded in a value must not terminate the request.
  r=request(ValidateJob);r.text(0x42,"job-name",std::string("x\3y",3));r.end();parser.reset();for(uint8_t b:r.bytes)result=parser.feed(b);assert(result==1&&parser.request().name.size()==3);
  Server server(writer,ready,flush,activity);server.begin();server.advertise("test");server.setPrinter(Pcl,"HP LaserJet P2015");
  r=request(GetPrinterAttributes);r.end();auto socket=send(r.bytes);run(server,socket);assert(responseStatus(*socket)==Ok);
  std::string attrs(socket->output.begin(),socket->output.end());assert(attrs.find("image/pwg-raster")!=attrs.npos);assert(attrs.find("image/urf")==attrs.npos);
  socket=send(printBody(),true,true);run(server,socket);assert(responseStatus(*socket)==Ok&&!server.busy()&&!active);
  // Queries and cancellation while a document is being received.
  auto body=printBody();socket=send(body);socket->input.resize(1024);for(int i=0;i<10;++i)server.poll();assert(server.busy());
  r=request(GetPrinterAttributes);r.end();auto query=send(r.bytes);run(server,query);assert(responseStatus(*query)==Ok);
  r=request(PrintJob);r.end();query=send(r.bytes);run(server,query);assert(responseStatus(*query)==PrinterBusy);
  r=request(CancelJob,2);r.end();query=send(r.bytes);run(server,query);assert(responseStatus(*query)==Ok&&!server.busy());run(server,socket);assert(responseStatus(*socket)==NotPossible);
  // Create-Job / Send-Document is used by some Android IPP clients.
  r=request(CreateJob);r.end();query=send(r.bytes);run(server,query);assert(server.busy());
  r=request(SendDocument,3);r.boolean("last-document",true);r.end();auto v=raster();r.bytes.insert(r.bytes.end(),v.begin(),v.end());query=send(r.bytes);run(server,query);assert(responseStatus(*query)==Ok&&!server.busy());
  r=request(CreateJob);r.end();query=send(r.bytes);run(server,query);testMillis+=90001;server.poll();assert(!server.busy());
  // Body truncation and tail USB failures must never return success.
  body=printBody();body.pop_back();query=send(body);run(server,query);assert(responseStatus(*query)==BadRequest&&!server.busy());
  flushOk=false;query=send(printBody());run(server,query);assert(responseStatus(*query)==InternalError&&!server.busy());flushOk=true;
  usbReady=false;query=send(printBody());run(server,query);assert(responseStatus(*query)==ServiceUnavailable);usbReady=true;
  // Conflicting framing must be rejected before USB ownership is acquired.
  r=request(GetPrinterAttributes);r.end();query=send(r.bytes,true);
  std::string raw(query->input.begin(),query->input.end());auto split=raw.find("\r\n\r\n");raw.insert(split,"\r\nContent-Length: 100");query->input.assign(raw.begin(),raw.end());run(server,query);
  std::string badHttp(query->output.begin(),query->output.end());assert(badHttp.find("400 Bad Request")!=badHttp.npos&&!server.busy());
  // Receive timeout releases ownership and allows the next job.
  query=send(printBody());query->input.resize(1024);for(int i=0;i<10;++i)server.poll();assert(server.busy());testMillis+=30001;server.poll();assert(!server.busy());
  // USB failure midway through a raster must not be reported as completed.
  usbOk=false;query=send(printBody());run(server,query);assert(responseStatus(*query)!=Ok&&!server.busy());usbOk=true;
  // ESC/P-R (Epson): colour page, exact pixels inside the 3 mm border, multi-page end flags.
  printed.clear();d.reset();d.setLanguage(Escpr);{auto v=std::vector<uint8_t>{'R','a','S','2'};auto pg=rgbPage();v.insert(v.end(),pg.begin(),pg.end());decode(v,d);}assert(d.finish());checkEscpr(1);
  printed.clear();d.reset();{auto v=std::vector<uint8_t>{'R','a','S','2'};auto pg=rgbPage();v.insert(v.end(),pg.begin(),pg.end());v.insert(v.end(),pg.begin(),pg.end());decode(v,d);}assert(d.finish()&&d.pages()==2);checkEscpr(2);
  // Grey input is sent as RGB with monochrome colour mode; 600 dpi is refused.
  printed.clear();d.reset();decode(raster(8,18),d);assert(d.finish());{std::string o(printed.begin(),printed.end());size_t q=o.find("setq");assert(q!=o.npos&&printed[q+4+2]==1);}
  d.reset();rejected=false;for(uint8_t b:raster(8,18,600)){if(!d.feed(b)){rejected=true;break;}}assert(rejected);
  // Aborting mid-page still closes the ESC/P-R job.
  printed.clear();d.reset();{auto v=raster(8,18);v.resize(4+1796+8);for(uint8_t b:v){while(d.pending())assert(d.drain());assert(d.feed(b));}}d.abort();
  {std::string o(printed.begin(),printed.end());assert(o.find("endp")!=o.npos&&o.find("endj")!=o.npos);}
  d.setLanguage(Pcl);
  r=request(ValidateJob);r.text(0x44,"print-color-mode","color");r.end();parser.reset();for(uint8_t b:r.bytes)result=parser.feed(b);assert(result==1&&parser.request().status==Ok);
  // An Epson advertises colour and 300 dpi only; unknown printers refuse phone jobs.
  server.setPrinter(Escpr,"EPSON L130 Series");r=request(GetPrinterAttributes);r.end();socket=send(r.bytes);run(server,socket);
  attrs.assign(socket->output.begin(),socket->output.end());assert(attrs.find("srgb_8")!=attrs.npos&&attrs.find("EPSON L130 Series")!=attrs.npos&&attrs.find("two-sided-long-edge")==attrs.npos);
  server.setPrinter(NoLanguage,"");query=send(printBody());run(server,query);assert(responseStatus(*query)==ServiceUnavailable);
  if(argc>1){std::ifstream f(argv[1],std::ios::binary);std::vector<uint8_t> external((std::istreambuf_iterator<char>(f)),{});assert(!external.empty());d.reset();decode(external,d);assert(d.finish());std::cout<<"External CUPS PWG fixture passed\n";}
  std::cout<<"PWG decoder (PCL + ESC/P-R), IPP codec, HTTP framing and job lifecycle tests passed\n";
}
