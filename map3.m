clear
pkg load instrument-control

%s1 = serial("COM9",  115200, 150);         %USB
s1 = serial("\\\\.\\COM13", 115200, 150);  %Bluetooth
nb_dom = 0;
nb_temp  = 0;

srl_flush(s1);

while (true)
  if (char(srl_read(s1, 1)) == 'S')
    if (char(srl_read(s1, 1)) == 'T')
      if (char(srl_read(s1, 1)) == 'A')
        if (char(srl_read(s1, 1)) == 'R')
          if (char(srl_read(s1, 1)) == 'T')                                     % get start signal
            disp('signal received');
            data_size = uint32(typecast(uint8(srl_read(s1, 2)), 'uint16'));             % get transmission size
            nb_dom = uint32(typecast(uint8(srl_read(s1, 2)), 'uint16'));
            nb_temp  = uint32(typecast(uint8(srl_read(s1, 2)), 'uint16'));
            buffer    = zeros(data_size, 1);

            dom_cnt    = 1;
            temp_cnt    = 1;
            cnt_dn     = 0;

            xd = zeros(nb_dom, 1);
            yd = zeros(nb_dom, 1);
            xt = zeros(nb_temp, 1);
            yt = zeros(nb_temp, 1);

            buffer = srl_read(s1, data_size);                                           %get transmission data

            printf("signal end\ndecoding: ");

            for n=1:data_size     %decode data
              if (cnt_dn > 0)
                cnt_dn--;
              elseif (buffer(n) == 'D')
                printf("X")
                xd(dom_cnt) = typecast(uint8([buffer(n+1 : n+2)]), 'int16');
                yd(dom_cnt) = typecast(uint8([buffer(n+3 : n+4)]), 'int16');
                dom_cnt++;
                cnt_dn = 4;
              elseif (buffer(n) == 'T')
                printf(".")
                xt(wl_cnt) = typecast(uint8([buffer(n+1 : n+2)]), 'int16');
                yt(wl_cnt) = typecast(uint8([buffer(n+3 : n+4)]), 'int16');
                dom_cnt++;
                cnt_dn = 4;
              else
                disp(" ! transmission error !");
                printf(char(buffer(n-1:n+1)));
                srl_flush(s1);
                break;
              endif
            endfor

            printf("\ndone\n");

            %figure
            plot(xd, yd, 'rd-', xt, yt, 'bx');
            axis("equal");

            pause(1);
            srl_flush(s1);
          endif % 'T'
        endif   % 'R'
      endif     % 'A'
    endif       % 'T'
  endif         % 'S'
endwhile
