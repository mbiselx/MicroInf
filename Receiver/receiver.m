clear

s1 = com_setup("\\\\.\\COM13");    % change this to suit your setup

nb_dom   = 0;
nb_temp  = 0;
conversion_factor = 1000/13;%  1000 steps / cm

com_flush(s1);

while (true)
  if (char(com_read(s1, 1)) == 'S')
    if (char(com_read(s1, 1)) == 'T')
      if (char(com_read(s1, 1)) == 'A')
        if (char(com_read(s1, 1)) == 'R')
          if (char(com_read(s1, 1)) == 'T')  % get start signal
            disp('signal received');
            data_size = uint32(typecast(uint8(com_read(s1, 2)), 'uint16')); % get transmission sizes
            nb_dom    = uint32(typecast(uint8(com_read(s1, 2)), 'uint16'));
            nb_temp   = uint32(typecast(uint8(com_read(s1, 2)), 'uint16'));
            buffer    = zeros(1, data_size);

            dom_cnt    = 0;                 % arrays start with 1 ...
            temp_cnt   = 0;
            cnt_dn     = 0;

            xd = zeros(1, nb_dom);
            yd = zeros(1, nb_dom);
            xt = zeros(1, nb_temp);
            yt = zeros(1, nb_temp);

            buffer = com_read(s1, data_size);   % get transmission data

            printf("signal end\ndecoding: ");

            for n=1:data_size     %decode data
              if (cnt_dn > 0)
                cnt_dn = cnt_dn - 1;  % what kind of programming language doesn't accept i-- and i++ ???????
              elseif (buffer(n) == 'D')
                printf("X")
                xd(dom_cnt+1) = typecast(uint8([buffer(n+1 : n+2)]), 'int16');
                yd(dom_cnt+1) = typecast(uint8([buffer(n+3 : n+4)]), 'int16');
                dom_cnt = dom_cnt + 1;
                cnt_dn = 4;
              elseif (buffer(n) == 'T')
                printf(".")
                xt(temp_cnt+1) = typecast(uint8([buffer(n+1 : n+2)]), 'int16');
                yt(temp_cnt+1) = typecast(uint8([buffer(n+3 : n+4)]), 'int16');
                temp_cnt = temp_cnt + 1;
                cnt_dn = 4;
              else
                disp(" ! transmission error !");
                printf(char(buffer(n:end)));
                com_flush(s1);
                break;
              end %if
            end %for

            printf("\ndone\n");

            figure(1)
            plot(xd/conversion_factor, yd/conversion_factor, 'rd-;dominant point;', xt/conversion_factor, yt/conversion_factor, 'bx;temporary point;');
            axis("equal");
            grid on;
            xlabel("position [cm]");
            ylabel("position [cm]");

            pause(.2);
            com_flush(s1);
          end %if % 'T'
        end %if   % 'R'
      end %if     % 'A'
    end %if       % 'T'
  end %if         % 'S'
end %while
