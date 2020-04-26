clear

s1 = com_setup("\\\\.\\COM13");    % change this to suit your setup

nb_dom = 0;
nb_temp  = 0;

com_flush(s1);

while (true)
  %printf(".");
  if (char(com_read(s1, 1)) == 'S')
    if (char(com_read(s1, 1)) == 'T')
      if (char(com_read(s1, 1)) == 'A')
        if (char(com_read(s1, 1)) == 'R')
          if (char(com_read(s1, 1)) == 'T')                                     % get start signal
            disp('signal received');
            data_size = uint32(typecast(uint8(com_read(s1, 2)), 'uint16'));             % get transmission size
            nb_dom = uint32(typecast(uint8(com_read(s1, 2)), 'uint16'));
            nb_temp  = uint32(typecast(uint8(com_read(s1, 2)), 'uint16'));
            buffer    = zeros(data_size, 1);

            dom_cnt    = 1;
            temp_cnt    = 1;
            cnt_dn     = 0;

            xd = zeros(nb_dom, 1);
            yd = zeros(nb_dom, 1);
            xt = zeros(nb_temp, 1);
            yt = zeros(nb_temp, 1);

            buffer = com_read(s1, data_size);                                           %get transmission data

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
                xt(temp_cnt) = typecast(uint8([buffer(n+1 : n+2)]), 'int16');
                yt(temp_cnt) = typecast(uint8([buffer(n+3 : n+4)]), 'int16');
                temp_cnt++;
                cnt_dn = 4;
              else
                disp(" ! transmission error !");
                printf(char(buffer(n-1:n+1)));
                com_flush(s1);
                break;
              end %if
            end %for

            printf("\ndone\n");

            figure(1)
            plot(xd, yd, 'rd-', xt, yt, 'bx');
            %xlim([min([xd', xt'])-10, max([xd', xt'])+10]);
            %ylim([min([yd', yt'])-10, max([yd', yt'])+10]);
            axis("equal");
            %axis("image");


            pause(1);
            com_flush(s1);
          end %if % 'T'
        end %if   % 'R'
      end %if     % 'A'
    end %if       % 'T'
  end %if         % 'S'
end %while
