function send_code(code){
  if(last_code == code) return
  last_code = code;
  var command = ''
  switch(code) {
    case 38: //up
      command = "UP"
    break;
    case 37: //lf
      command = "LF"
    break;
    case 39: //rh
      command = "RH"
    break;
    case 40: //dw
      command = "DW"
    break;
    case 0: //st
      command = "ST"
    break;
    default:
      return;
    }
  $("#demo").text("The Unicode value is: " + inx + ' ' + code);
  $.get("pad/"+command, function(data, status){$("#data").text(data)});
  inx = inx + 1;
}

function  send_stop(){
  if(last_code == 0) return
  document.getElementById("demo").innerHTML = "stop " + inx;
  send_code(0)
  inx = inx + 1;
}
