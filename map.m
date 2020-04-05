pkg load instrument-control

%fclose(s1);
%s1 = serial("COM9");         %USB-Michael
s1 = serial("\\\\.\\COM13");  %Bluetooth-Michael+Samuel
set(s1, 'baudrate', 115200, 'bytesize', 8, 'timeout', 150);
nb_pts = 0;

srl_flush(s1);


while (true)
  if (char(srl_read(s1, 1)) == 'S')           % get start signal
    disp('signal received');
    data_size = uint32(typecast(uint8(srl_read(s1, 2)), 'uint16'));
    nb_pts    = uint32(typecast(uint8(srl_read(s1, 2)), 'uint16'));
    nb_walls  = uint32(typecast(uint8(srl_read(s1, 2)), 'uint16'));      % get transmission size
    buffer = zeros(4*nb_pts, 1);

    if (nb_pts > 0 && nb_pts < 65000)
      buffer = srl_read(s1, 5*nb_pts);    % recieve data

      x = zeros(nb_pts, 1);
      y = zeros(nb_pts, 1);

      for n = 0:nb_pts-1
        x(n+1) =  typecast(uint8([buffer(5*n+2), buffer(5*n+3)]), 'int16');
        y(n+1) =  typecast(uint8([buffer(5*n+4), buffer(5*n+5)]), 'int16');
      endfor

      plot(x,y, 'bx');
      pause(.5);
      srl_flush(s1);
    endif
  endif
endwhile
