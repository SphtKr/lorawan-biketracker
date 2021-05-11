function Decode(fPort, bytes, variables) {

  // Based on https://www.thethingsnetwork.org/forum/t/another-payload-decoder-question-issues-converting-bytes-to-float/43879
  // Based on https://stackoverflow.com/a/37471538 by Ilya Bursov
  function bytesToFloat(bytes) {
    // JavaScript bitwise operators yield a 32 bits integer, not a float.
    // Assume LSB (least significant byte first).
    var bits = bytes[3]<<24 | bytes[2]<<16 | bytes[1]<<8 | bytes[0];
    var sign = (bits>>>31 === 0) ? 1.0 : -1.0;
    var e = bits>>>23 & 0xff;
    var m = (e === 0) ? (bits & 0x7fffff)<<1 : (bits & 0x7fffff) | 0x800000;
    var f = sign * m * Math.pow(2, e - 150);
    return f;
  } 

  var hdop = bytesToFloat(bytes.slice(12,16));
  var inactive_samples = bytes[16];
  var shocks = bytes[17];
  var battery_raw = bytes[18]*256+bytes[19]
  return {
    "latitude": bytesToFloat(bytes.slice(0,4)),
  	"longitude": bytesToFloat(bytes.slice(4,8)),
    "altitude": bytesToFloat(bytes.slice(8,12)),
    "hdop": hdop,
    "inactive_samples": inactive_samples,
    "shocks": shocks, 
    "battery_raw": battery_raw,
    "battery_level": 100*(battery_raw-2480)/(4108-2480),
    "gps_accuracy": 6*hdop // Real calculation of this needs more info, this one based on Wikipedia
  };
}

