// NEW: Helper function to decode manufacturer data
std::string decode_manufacturer_data(uint16_t company_id, const std::vector<uint8_t>& data) {
  switch (company_id) {
    case 0x004C: { // Apple
      if (data.size() < 2) return "Apple (Too Short)";
      uint8_t type = data[0];
      // See: https://github.com/furiousMAC/continuity/blob/master/messages/apple/nearby.md
      switch (type) {
        case 0x02: return "Apple Device (iPhone/iPad/Mac)";
        case 0x03: return "Apple Device (Watch)";
        case 0x05: return "Apple Device (AirPods)";
        case 0x07: return "Apple Device (HomePod)";
        case 0x09: return "Apple Device (AirTag)";
        case 0x0C: return "Apple Find My";
        case 0x10: return "Apple AirDrop";
        default: return "Apple (Unknown Type)";
      }
      break;
    }
    case 0x0075: { // Samsung
      if (data.size() < 1) return "Samsung (Too Short)";
      uint8_t type = data[0];
      if (type == 0x54) {
          return "Samsung SmartThings";
      }
      return "Samsung (Unknown Type)";
    }
    // Add other cases for manufacturers like Google (0x00E0) etc. here
    default:
      return ""; // No specific decoder available
  }
  return "";
}