function ab2v(p0,p1,p2, fhandle)
        
        p1 = p1/norm(p1);
        p2 = p2/norm(p2);
        
        axes(fhandle);
        
        cla
        
        grid on
        
        xlabel('x')
        ylabel('y')
        zlabel('z')
                
        x0 = p0(1);
        y0 = p0(2);
        x1 = p1(1);
        y1 = p1(2);
        x2 = p2(1);
        y2 = p2(2);
        
        d1 = p1-p0;
        d2 = p2-p0;
        
        alpha = 0.03;  % Size of arrow head relative to the length of the vector
        beta = 0.2;  % Width of the base of the arrow head relative to the length
        
        hu1 = [x1-alpha*(d1(1)+beta*(d1(2)+eps)); x1; x1-alpha*(d1(1)-beta*(d1(2)+eps))];
        hv1 = [y1-alpha*(d1(2)-beta*(d1(1)+eps)); y1; y1-alpha*(d1(2)+beta*(d1(1)+eps))];

        hu2 = [x2-alpha*(d2(1)+beta*(d2(2)+eps)); x2; x2-alpha*(d2(1)-beta*(d2(2)+eps))];
        hv2 = [y2-alpha*(d2(2)-beta*(d2(1)+eps)); y2; y2-alpha*(d2(2)+beta*(d2(1)+eps))]; 
        
        
        plot([x0;x1],[y0;y1]);   % Draw a line between p0 and p1
        hold on
        plot(hu1(:),hv1(:))  % Plot arrow head
       
        
        plot([x0;x2],[y0;y2]);   % Draw a line between p0 and p1
        plot(hu2(:),hv2(:))  % Plot arrow head

        hold off
     
end