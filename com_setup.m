%% subfunction that sets up a blutooth serial connection
function s = com_setup(port)

  if (is_octave)
    pkg load instrument-control
    s = serial(port, 115200, 150);
  else
    s = serial(port, 'BaudRate', 115200);
    fopen(s)
    s.ReadAsyncMode = 'manual';
  end

end
