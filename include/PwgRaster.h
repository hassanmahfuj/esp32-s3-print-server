#pragma once
// Streaming PWG Raster -> printer language, A4 only.
//   PCL 5   (HP LaserJet style): 300/600 dpi, monochrome, simplex or duplex.
//   ESC/P-R (Epson inkjet):      300 dpi, colour or grey, simplex.
// Format reference: PWG 5102.4-2012, sections 4.2-4.4.
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

namespace mobileprint {
inline uint32_t be32(const uint8_t *p) {
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) |
         (uint32_t(p[2]) << 8) | p[3];
}

enum Language : uint8_t { NoLanguage, Pcl, Escpr };

class PwgRaster {
 public:
  typedef bool (*Writer)(const uint8_t *, size_t);
  explicit PwgRaster(Writer writer) : writer_(writer) { reset(); }
  void reset() {
    state_ = Sync; pos_ = rowPos_ = row_ = pages_ = repeat_ = run_ = unitPos_ = 0;
    started_ = pageOpen_ = false; error_ = nullptr;
  }
  const char *error() const { return error_; }
  bool pending() const { return state_ == Emit; }
  uint32_t pages() const { return pages_; }
  bool started() const { return started_; }
  void setLanguage(Language language) { language_ = language; }
  void setDuplex(const char *sides) {
    duplex_ = sides && strcmp(sides, "two-sided-long-edge") == 0 ? 1 :
              sides && strcmp(sides, "two-sided-short-edge") == 0 ? 2 : 0;
  }

  // Caller must drain pending rows before feeding more input. Never retain a page.
  bool feed(uint8_t byte) {
    if (error_) return false;
    switch (state_) {
      case Sync:
        if (byte != uint8_t("RaS2"[pos_++])) return fail("Not PWG Raster (RaS2)");
        if (pos_ == 4) { pos_ = 0; state_ = Header; }
        break;
      case Header:
        header_[pos_++] = byte;
        if (pos_ == sizeof(header_)) {
          pos_ = 0;
          if (!openPage()) return false;
          row_ = 0; state_ = Repeat;
        }
        break;
      case Repeat:
        repeat_ = uint16_t(byte) + 1;
        if (repeat_ > height_ - row_) return fail("Row repeat exceeds page");
        rowPos_ = 0; state_ = Control;
        break;
      case Control:
        // PWG: 0..127 repeat count+1, 129..255 literal count 257-control,
        // counted in pixels (bytes for 1-bit data).
        // 128 is not a valid PWG run (unlike extensions in some CUPS readers).
        if (byte == 128) return fail("Invalid PWG run");
        literal_ = byte > 128;
        run_ = literal_ ? 257 - byte : byte + 1;
        if (size_t(run_) * unit_ > bytesPerLine_ - rowPos_) return fail("Run exceeds row");
        unitPos_ = 0; state_ = Value;
        break;
      case Value:
        if (literal_) {
          rowData_[rowPos_++] = byte;
          if (++unitPos_ == unit_) { unitPos_ = 0; --run_; }
        } else {
          rowData_[rowPos_ + unitPos_] = byte;
          if (++unitPos_ == unit_) {
            for (uint16_t i = 1; i < run_; ++i)
              memcpy(rowData_ + rowPos_ + i * unit_, rowData_ + rowPos_, unit_);
            rowPos_ += run_ * unit_; run_ = 0; unitPos_ = 0;
          }
        }
        if (!run_) state_ = rowPos_ == bytesPerLine_ ? Emit : Control;
        break;
      case Emit: return fail("Decoder must drain before input");
    }
    return true;
  }

  // One physical row per call, so HTTP queries can run between USB writes.
  bool drain() {
    if (error_) return false;
    if (state_ != Emit) return true;
    if (!(language_ == Escpr ? escprRow() : pclRow())) return false;
    ++row_;
    if (--repeat_) return true;
    if (row_ == height_) {
      // ESC/P-R needs to know whether another page follows, so its
      // end-of-page goes out with the next header or with finish().
      if (language_ == Pcl && !text("\033*rB\f")) return false;
      ++pages_; state_ = Header;
    } else state_ = Repeat;
    return true;
  }

  bool finish() {
    if (error_) return false;
    if (!pages_ || state_ != Header || pos_) return fail("Truncated PWG document");
    if (language_ == Escpr) return escprEndJob();
    // Close the PCL language and the PJL job explicitly.  Some LaserJet
    // firmware keeps the USB status LED blinking until it sees EOJ/UEL.
    return text("\033E\033%-12345X@PJL EOJ NAME=\"IPP\"\r\n\033%-12345X");
  }
  void abort() {
    // A partial page may already have printed. Never resend failed USB bytes.
    if (started_) {
      if (language_ == Escpr) { error_ = nullptr; escprEndJob(); }
      else {
        const char *end = "\033*rB\033E\033%-12345X@PJL EOJ NAME=\"IPP\"\r\n\033%-12345X";
        writer_(reinterpret_cast<const uint8_t *>(end), strlen(end));
      }
    }
    reset();
  }

 private:
  // ESC/P-R page geometry at 300 dpi: A4 with the printer's 3 mm border.
  static const uint32_t EscprPaperW = 2480, EscprPaperH = 3507, EscprMargin = 35;
  static const uint32_t EscprAreaW = 2410, EscprAreaH = 3437;

  enum State { Sync, Header, Repeat, Control, Value, Emit } state_;
  Writer writer_;
  Language language_ = Pcl;
  uint8_t header_[1796], rowData_[2481 * 3], bits_[621];
  uint8_t line_[EscprAreaW * 3], packed_[EscprAreaW * 3];
  uint32_t dpi_ = 300, margin_ = 75; // 6.35 mm: dpi / 4
  uint32_t width_ = 0, height_ = 0, depth_ = 0, bpp_ = 0, colorSpace_ = 0;
  uint32_t bytesPerLine_ = 0, row_, pages_;
  size_t pos_, rowPos_;
  uint16_t repeat_, run_;
  uint8_t unit_ = 1, unitPos_;
  bool literal_ = false, started_, pageOpen_;
  uint8_t duplex_ = 0;
  const char *error_;
  bool fail(const char *s) { error_ = s; return false; }
  bool write(const uint8_t *p, size_t n) { return writer_(p, n) || fail("USB write failed"); }
  bool text(const char *s) { return write(reinterpret_cast<const uint8_t *>(s), strlen(s)); }

  bool openPage() {
    width_ = be32(header_ + 372); height_ = be32(header_ + 376);
    depth_ = be32(header_ + 384); bpp_ = be32(header_ + 388);
    colorSpace_ = be32(header_ + 400); bytesPerLine_ = be32(header_ + 392);
    dpi_ = be32(header_ + 276); margin_ = dpi_ / 4;
    unit_ = bpp_ >= 8 ? uint8_t(bpp_ / 8) : 1;
    if (pages_ >= 1000) return fail("Too many pages");
    return language_ == Escpr ? escprOpenPage() : pclOpenPage();
  }
  bool a4(uint32_t dpi) const {
    return dpi == 300
      ? width_ >= 2480 && width_ <= 2481 && height_ >= 3507 && height_ <= 3508
      : dpi == 600 && width_ >= 4960 && width_ <= 4961 && height_ >= 7015 && height_ <= 7016;
  }
  // CUPS/PWG sets Duplex at +272 and Tumble at +368 for two-sided jobs.
  // They are boolean fields; accept either value and let the IPP sides
  // attribute select the binding direction.
  bool commonHeaderOk() const {
    return be32(header_ + 272) <= 1 && be32(header_ + 368) <= 1 && be32(header_ + 340) <= 1 &&
           !be32(header_ + 344) && be32(header_ + 456) <= 1 && be32(header_ + 460) <= 1 &&
           !be32(header_ + 396) && be32(header_ + 420) == 1 &&
           bytesPerLine_ == (width_ * bpp_ + 7) / 8;
  }

  // ---------------- PCL 5 ----------------
  bool pclOpenPage() {
    if (!a4(dpi_) || be32(header_ + 280) != dpi_)
      return fail("Only full-page A4 at 300 or 600 dpi is supported");
    if (!commonHeaderOk() || bpp_ != depth_ ||
        !((depth_ == 1 && (colorSpace_ == 3 || colorSpace_ == 18)) ||
          (depth_ == 8 && colorSpace_ == 18)))
      return fail("Unsupported PWG page settings");
    if (!started_) {
      // PCL 5 only. No PDF/PostScript interpreter on the ESP32.
      const char *pjlDuplex = duplex_ == 1
        ? "@PJL SET DUPLEX=ON\n@PJL SET BINDING=LONGEDGE\n"
        : duplex_ == 2
          ? "@PJL SET DUPLEX=ON\n@PJL SET BINDING=SHORTEDGE\n"
          : "@PJL SET DUPLEX=OFF\n";
      char start[220];
      int startLen = snprintf(start, sizeof(start),
        "\033%%-12345X@PJL JOB NAME=\"IPP\"\n%s@PJL ENTER LANGUAGE=PCL\n\033E", pjlDuplex);
      if (startLen <= 0 || !write(reinterpret_cast<const uint8_t *>(start), size_t(startLen))) return false;
      started_ = true;
    }
    char command[180];
    // Duplex is a job-level setting. Repeating it before page 2 can make
    // older LaserJet firmware start a new physical sheet instead of using
    // the back side of the current one.
    const char *duplexCommand = pages_ ? "" :
      (duplex_ == 1 ? "\033&l1S" : duplex_ == 2 ? "\033&l2S" : "\033&l0S");
    const char *jobPageSetup = pages_ ? "" : "\033&l26A\033&l0O\033&l0E\033&l0L\033&l1X";
    // PCL logical left edge is 1/4 inch from physical paper; crop that margin.
    // Top margin zero, then move down 1/4 inch in the selected device units.
    int n = snprintf(command, sizeof(command),
      "%s%s"
      "\033&a0L\033&u%uD\033*p0X\033*p%uY\033*t%uR\033*r%uS\033*r%uT"
      "\033*r1A\033*b0M", jobPageSetup, duplexCommand,
      unsigned(dpi_), unsigned(margin_), unsigned(dpi_),
      unsigned(width_ - 2 * margin_), unsigned(height_ - 2 * margin_));
    pageOpen_ = true;
    return write(reinterpret_cast<const uint8_t *>(command), size_t(n));
  }
  bool pclRow() {
    if (row_ < margin_ || row_ >= height_ - margin_) return true;
    const uint32_t outWidth = width_ - 2 * margin_;
    const size_t count = (outWidth + 7) / 8;
    memset(bits_, 0, count);
    static const uint8_t bayer[4][4] = {
      {0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}
    };
    for (uint32_t x = 0; x < outWidth; ++x) {
      const uint32_t src = x + margin_;
      bool black;
      if (depth_ == 1) {
        bool bit = (rowData_[src / 8] & (0x80 >> (src & 7))) != 0;
        black = colorSpace_ == 3 ? bit : !bit; // DeviceK vs sGray
      } else {
        uint8_t gray = rowData_[src];
        black = gray < uint8_t(bayer[row_ & 3][src & 3] * 16 + 8);
      }
      if (black) bits_[x / 8] |= 0x80 >> (x & 7);
    }
    char command[32];
    int n = snprintf(command, sizeof(command), "\033*b%uW", unsigned(count));
    return write(reinterpret_cast<const uint8_t *>(command), size_t(n)) && write(bits_, count);
  }

  // ---------------- ESC/P-R ----------------
  // Commands are ESC, a class letter, a little-endian 32-bit parameter
  // length, a 4-letter name, then big-endian parameters.
  bool escprCommand(char cls, const char *name, const uint8_t *params, size_t n) {
    uint8_t head[10] = {0x1B, uint8_t(cls), uint8_t(n), uint8_t(n >> 8), uint8_t(n >> 16), uint8_t(n >> 24)};
    memcpy(head + 6, name, 4);
    return write(head, sizeof(head)) && (!n || write(params, n));
  }
  static void put16(uint8_t *p, uint32_t v) { p[0] = uint8_t(v >> 8); p[1] = uint8_t(v); }
  static void put32(uint8_t *p, uint32_t v) { put16(p, v >> 16); put16(p + 2, v); }

  bool escprStartJob() {
    static const uint8_t start[] =
      // Leave IEEE 1284.4 packet mode, reset.
      "\x00\x00\x00\x1B\x01@EJL 1284.4\n@EJL     \n"
      "\x1B@"
      // Remote mode: job start, HD, paper path auto; then exit remote mode.
      "\x1B(R\x08\x00\x00REMOTE1"
      "JS\x04\x00\x00\x00\x00\x00"
      "HD\x03\x00\x00\x03\xFF"
      "PP\x03\x00\x00\x00\x00"
      "\x1B\x00\x00\x00"
      // Enter ESC/P-R raster mode.
      "\x1B(R\x06\x00\x00""ESCPR";
    if (!write(start, sizeof(start) - 1)) return false;
    // Plain paper, normal quality, colour or monochrome, no adjustments, full colour RGB.
    const uint8_t quality[9] = {0, 1, uint8_t(colorSpace_ == 19 ? 0 : 1), 0, 0, 0, 0, 0, 0};
    const uint8_t imaging[10] = {0};
    uint8_t job[22];
    put32(job, EscprPaperW); put32(job + 4, EscprPaperH);
    put16(job + 8, EscprMargin); put16(job + 10, EscprMargin);
    put32(job + 12, EscprAreaW); put32(job + 16, EscprAreaH);
    job[20] = 2; // input resolution 300x300
    job[21] = 0; // bidirectional printing
    return escprCommand('q', "setq", quality, sizeof(quality)) &&
           escprCommand('q', "seti", imaging, sizeof(imaging)) &&
           escprCommand('j', "setj", job, sizeof(job));
  }
  bool escprEndPage(bool nextPage) {
    const uint8_t flag = nextPage ? 1 : 0;
    pageOpen_ = false;
    return escprCommand('p', "endp", &flag, 1);
  }
  bool escprEndJob() {
    static const uint8_t end[] =
      "\x1B@"
      "\x1B(R\x08\x00\x00REMOTE1"
      "LD\x00\x00"
      "JE\x01\x00\x00"
      "\x1B\x00\x00\x00";
    return (!pageOpen_ || escprEndPage(false)) &&
           escprCommand('j', "endj", nullptr, 0) && write(end, sizeof(end) - 1);
  }
  bool escprOpenPage() {
    if (!a4(300) || dpi_ != 300 || be32(header_ + 280) != 300)
      return fail("Only full-page A4 at 300 dpi is supported");
    if (!commonHeaderOk() ||
        !((depth_ == 1 && bpp_ == 1 && (colorSpace_ == 3 || colorSpace_ == 18)) ||
          (depth_ == 8 && bpp_ == 8 && colorSpace_ == 18) ||
          (depth_ == 8 && bpp_ == 24 && colorSpace_ == 19)))
      return fail("Unsupported PWG page settings");
    if (!started_) {
      if (!escprStartJob()) return false;
      started_ = true;
    } else if (pageOpen_ && !escprEndPage(true)) return false;
    pageOpen_ = true;
    return escprCommand('p', "sttp", nullptr, 0);
  }
  // PWG rows cover the whole sheet; send only the part inside the printable area.
  bool escprRow() {
    if (row_ < EscprMargin || row_ >= EscprMargin + EscprAreaH) return true;
    for (uint32_t x = 0; x < EscprAreaW; ++x) {
      const uint32_t src = x + EscprMargin;
      uint8_t *out = line_ + x * 3;
      if (bpp_ == 24) { memcpy(out, rowData_ + src * 3, 3); continue; }
      uint8_t gray;
      if (bpp_ == 8) gray = rowData_[src];
      else {
        bool bit = (rowData_[src / 8] & (0x80 >> (src & 7))) != 0;
        gray = (colorSpace_ == 3 ? bit : !bit) ? 0 : 255;
      }
      out[0] = out[1] = out[2] = gray;
    }
    size_t n = packRow();
    const bool packed = n != 0;
    if (!packed) n = sizeof(line_);
    uint8_t params[7];
    put16(params, 0); put16(params + 2, row_ - EscprMargin);
    params[4] = packed ? 1 : 0;
    put16(params + 5, uint32_t(n));
    const uint32_t len = 7 + uint32_t(n);
    uint8_t head[10] = {0x1B, 'd', uint8_t(len), uint8_t(len >> 8), uint8_t(len >> 16), uint8_t(len >> 24), 'd', 's', 'n', 'd'};
    return write(head, sizeof(head)) && write(params, sizeof(params)) &&
           write(packed ? packed_ : line_, n);
  }
  // Run-length encodes line_ (RGB pixels) into packed_. Returns 0 when it would
  // not be smaller than the raw row. Repeat run of n pixels (2..129): 257-n, pixel.
  // Literal run of n pixels (1..128): n-1, pixels.
  size_t packRow() {
    const size_t count = EscprAreaW, limit = sizeof(packed_);
    auto same = [&](size_t a, size_t b) { return memcmp(line_ + a * 3, line_ + b * 3, 3) == 0; };
    size_t i = 0, o = 0;
    while (i < count) {
      size_t r = 1;
      while (i + r < count && r < 129 && same(i, i + r)) ++r;
      if (r >= 2) {
        if (o + 4 > limit) return 0;
        packed_[o++] = uint8_t(257 - r);
        memcpy(packed_ + o, line_ + i * 3, 3); o += 3; i += r;
        continue;
      }
      size_t l = 1;
      while (i + l < count && l < 128 && !(i + l + 1 < count && same(i + l, i + l + 1))) ++l;
      if (o + 1 + l * 3 > limit) return 0;
      packed_[o++] = uint8_t(l - 1);
      memcpy(packed_ + o, line_ + i * 3, l * 3); o += l * 3; i += l;
    }
    return o < limit ? o : 0;
  }
};
} // namespace mobileprint
