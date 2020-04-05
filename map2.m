clear
pkg load instrument-control

%s1 = serial("COM9",  115200, 150);         %USB
s1 = serial("\\\\.\\COM13", 115200, 150);  %Bluetooth
nb_points = 0;
nb_walls  = 0;

srl_flush(s1);

while (true)
  if (char(srl_read(s1, 1)) == 'S')
    if (char(srl_read(s1, 1)) == 'T')
      if (char(srl_read(s1, 1)) == 'A')
        if (char(srl_read(s1, 1)) == 'R')
          if (char(srl_read(s1, 1)) == 'T')                                     % get start signal
            disp('signal received');
            data_size = uint32(typecast(uint8(srl_read(s1, 2)), 'uint16'));             % get transmission size
            nb_points = uint32(typecast(uint8(srl_read(s1, 2)), 'uint16'));
            nb_walls  = uint32(typecast(uint8(srl_read(s1, 2)), 'uint16'));
            buffer    = zeros(data_size, 1);

            pt_cnt    = 1;
            wl_cnt    = 1;
            pflag     = 0;
            wflag     = 0;

            xp = zeros(nb_points, 1);
            yp = zeros(nb_points, 1);
            a  = zeros(nb_walls, 1);
            b  = zeros(nb_walls, 1);
            xw = zeros(nb_walls, 1);
            yw = zeros(nb_walls, 1);

            buffer = srl_read(s1, data_size);                                           %get transmission data

            printf("signal end\ndecoding: ");

            for n=1:data_size     %decode data
              if (pflag > 0)
                pflag--;
              elseif (wflag > 0)
                wflag--;
              elseif (buffer(n) == 'P')
                printf(".")
                xp(pt_cnt) = typecast(uint8([buffer(n+1), buffer(n+2)]), 'int16');
                yp(pt_cnt) = typecast(uint8([buffer(n+3), buffer(n+4)]), 'int16');
                pt_cnt++;
                pflag = 4;
              elseif (buffer(n) == 'W')
                printf("|")
                a(wl_cnt)  = typecast(uint8([buffer(n+ 1 : n+ 4)]), 'single');
                b(wl_cnt)  = typecast(uint8([buffer(n+ 5 : n+ 8)]), 'single');
                xw(wl_cnt) = typecast(uint8([buffer(n+ 9 : n+10)]), 'int16');
                yw(wl_cnt) = typecast(uint8([buffer(n+11 : n+12)]), 'int16');
                wl_cnt++;
                wflag = 12;
              else
                disp(" ! transmission error !");
                printf(char(buffer(n:n+1)));
                srl_flush(s1);
                break;
              endif
            endfor

            printf("\ndone\n")

            %figure
            plot(xp/100, yp/100, 'bx', xw/100, yw/100, 'rd-');
            axis("equal")

            %if (nb_walls > 0)
            %  x = [ min([xp; xw]) : max([xp; xw]) ];
            %  y = a + b.*x;
            %  hold on
            %  plot (x/100,y/100, 'g');
            %  axis([min([xp; xw]) , max([xp; xw])], "equal");
            %  hold off
            %end

            pause(1);
            srl_flush(s1);
          endif % 'T'
        endif   % 'R'
      endif     % 'A'
    endif       % 'T'
  endif         % 'S'
endwhile
