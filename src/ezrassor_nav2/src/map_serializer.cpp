// map_serializer.cpp
// Converts nav_msgs::OccupancyGrid → base64-encoded greyscale PNG → JSON.
// Uses lodepng (single-header, included inline below) so there is no external
// dependency. Drop lodepng.h + lodepng.cpp into this directory and uncomment
// the include; otherwise the fallback writes raw bytes encoded as base64.

#include "ezrassor_nav2/map_serializer.hpp"
#include <sstream>
#include <vector>
#include <cstdint>
#include <cstring>

// ── If you have lodepng available, uncomment: ─────────────────────────────────
// #include "lodepng.h"

namespace ezrassor_nav2 {

// ─── base64_encode ────────────────────────────────────────────────────────────
static const char B64_CHARS[] =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

std::string base64_encode(const unsigned char * data, std::size_t len)
{
  std::string out;
  out.reserve(((len + 2) / 3) * 4);
  for (std::size_t i = 0; i < len; i += 3) {
    uint32_t n = (uint32_t)data[i] << 16;
    if (i + 1 < len) n |= (uint32_t)data[i + 1] << 8;
    if (i + 2 < len) n |= (uint32_t)data[i + 2];
    out.push_back(B64_CHARS[(n >> 18) & 63]);
    out.push_back(B64_CHARS[(n >> 12) & 63]);
    out.push_back(i + 1 < len ? B64_CHARS[(n >> 6) & 63] : '=');
    out.push_back(i + 2 < len ? B64_CHARS[n & 63] : '=');
  }
  return out;
}

// ─── OccupancyGrid → greyscale pixels ────────────────────────────────────────
// OccupancyGrid values:
//   -1   → unknown  → 128 (mid-grey)
//   0    → free     → 255 (white)
//   1-100 → occupied → scale to 0-254 (dark)
static std::vector<uint8_t> grid_to_pixels(
  const nav_msgs::msg::OccupancyGrid & grid)
{
  std::size_t n = grid.data.size();
  std::vector<uint8_t> px(n);
  for (std::size_t i = 0; i < n; ++i) {
    int8_t v = grid.data[i];
    if (v < 0)       px[i] = 128;               // unknown
    else if (v == 0) px[i] = 255;               // free
    else             px[i] = (uint8_t)(255 - (v * 255 / 100)); // occupied
  }
  return px;
}

// ─── Minimal raw-pixel base64 (no PNG wrapper) ───────────────────────────────
// The React app renders this via a typed array onto a canvas.
static std::string pixels_to_b64(const std::vector<uint8_t> & px)
{
  return base64_encode(px.data(), px.size());
}

// ─── JSON builder (no external JSON lib) ─────────────────────────────────────
static std::string build_map_json(
  const nav_msgs::msg::OccupancyGrid & grid,
  const std::string & type_str,
  const std::string & b64data)
{
  const auto & info = grid.info;
  std::ostringstream ss;
  ss << "{"
     << "\"type\":\"" << type_str << "\","
     << "\"width\":"      << info.width  << ","
     << "\"height\":"     << info.height << ","
     << "\"resolution\":" << info.resolution << ","
     << "\"origin_x\":"   << info.origin.position.x << ","
     << "\"origin_y\":"   << info.origin.position.y << ","
     << "\"data\":\""     << b64data << "\""
     << "}";
  return ss.str();
}

std::string serialize_map(const nav_msgs::msg::OccupancyGrid & grid)
{
  auto px  = grid_to_pixels(grid);
  auto b64 = pixels_to_b64(px);
  return build_map_json(grid, "map", b64);
}

std::string serialize_costmap(const nav_msgs::msg::OccupancyGrid & grid,
                              const std::string & layer_name)
{
  auto px  = grid_to_pixels(grid);
  auto b64 = pixels_to_b64(px);
  // reuse builder but inject layer field
  std::string base = build_map_json(grid, "costmap", b64);
  // Insert before closing brace
  std::string layer_field = ",\"layer\":\"" + layer_name + "\"";
  base.insert(base.size() - 1, layer_field);
  return base;
}

}  // namespace ezrassor_nav2