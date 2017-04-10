// Example payload
// 04 73 28 00 
// 05 67 00 AA

// #define LPP_TEMPERATURE         103     //  0x67 - 103 - 2 bytes, 0.1°C signed
// #define LPP_BAROMETRIC_PRESSURE 115     // 0x73 - 115 - 2 bytes 0.1 hPa Unsigned 
  
function Decoder(bytes, port) {
  var sensor1_chan = bytes[0];
  var sensor1_type = bytes[1];
  var sensor1_type_name = "";
  var sensor1_value =  ((bytes[2] << 8) | bytes[3]); 
  sensor1_value = sensor1_value / 10;
  if (sensor1_type == 115){ sensor1_type_name = "Barometer";} else 
  if (sensor1_type == 103){ sensor1_type_name = "Temperature";} else 
  {                         sensor1_type_name = "unknown";}

  var sensor2_chan = bytes[4];
  var sensor2_type = bytes[5];
  var sensor2_type_name = "";
  var sensor2_value =  ((bytes[6] << 8) | bytes[7]); 
  sensor2_value = sensor2_value / 10;
  if (sensor2_type == 115){ sensor2_type_name = "Barometer";} else 
  if (sensor2_type == 103){ sensor2_type_name = "Temperature";} else 
  {                         sensor2_type_name = "unknown";}

  return {
    channel_1 : sensor1_chan,
    type_1: sensor1_type,
    type_name_1: sensor1_type_name,
    value_1: sensor1_value,
    
    channel_2 : sensor2_chan,
    type_2: sensor2_type,
    type_name_2: sensor2_type_name,
    value_2: sensor2_value
    
  };
}
