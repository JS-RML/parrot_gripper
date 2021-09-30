classdef FiveBarJaw < handle
    properties 
        %jaw id
        LR %0:left, 1:right
        graph %plot graph object
        % joint angle
        Q1
        Q2
        q3
        q4
        q5
        % joint coordinate
        P1
        P2
        P3
        P4
        P5
        % plot object
        link1
        link2
        link3
        link4
        link5
        link6
        link7
        joint1
        joint2
        joint3
        joint4
        joint5
        
        ft % fingertip coordinate
        L4_check %verify ik solution
        f2_check
    end

    methods
        function this = FiveBarJaw(LR,para,graph)
            this.LR = LR;
            this.graph = graph;
            if LR == 0 
                this.q5 = deg2rad(para.q5_offset+180);
                this.P5 = [-para.Ori_offsetX -para.Ori_offsetY];
            else
                this.q5 = deg2rad(-para.q5_offset);
                this.P5 = [para.Ori_offsetX -para.Ori_offsetY];
            end
            this.P1 = para.L5 * [cos(this.q5) sin(this.q5)] + this.P5;

            hold on;
            P_null = [nan nan];
            this.link1 = this.drawLink(P_null,P_null,'k');
            this.link2 = this.drawLink(P_null,P_null,'k');
            this.link3 = this.drawLink(P_null,P_null,'k');
            this.link4 = this.drawLink(P_null,P_null,'k');
            this.link5 = this.drawLink(P_null,P_null,'k');
            this.link6 = this.drawLink(P_null,P_null,'-.k');
            this.link7 = this.drawLink(P_null,P_null,'-.k');
            this.joint1 = plot(nan,nan,'.r','MarkerSize',10);
            this.joint2 = plot(nan,nan,'.r','MarkerSize',10);
            this.joint3 = plot(nan,nan,'.r','MarkerSize',10);
            this.joint4 = plot(nan,nan,'.r','MarkerSize',10);
            this.joint5 = plot(nan,nan,'.r','MarkerSize',10);
        end

        function compute_joint_pos(this,para,q1,q21)
            if this.LR == 0
                f1 = para.f1_L; %f2 = para.f2_L;
                q2 = deg2rad(q21-180+rad2deg(q1));
                phi = para.phi_L;
            else
                f1 = para.f1_R; %f2 = para.f2_L;
                q2 = deg2rad(180-q21+rad2deg(q1));
                phi = para.phi_R;
            end
            this.Q1 = q1; this.Q2 = q2;
            L1 = para.L1;
            L2 = para.L2;
            L3 = para.L3;
            L4 = para.L4;
            L5 = para.L5;
            this.P2 = L1 * [cos(q1) sin(q1)] + this.P1;
            this.P3 = L2 * [cos(q2) sin(q2)] + this.P2;

            % solve Q3
            C12 = L1*cos(q1)+L2*cos(q2);
            S12 = L1*sin(q1)+L2*sin(q2);
            A = -2*L3*(L5*cos(this.q5)+C12);
            B = -2*L3*(L5*sin(this.q5)+S12);
            C = L1^2+L2^2+L3^2-L4^2+L5^2 + 2*L5*(cos(this.q5)*C12+sin(this.q5)*S12) + 2*L1*L2*(cos(q1)*cos(q2)+sin(q1)*sin(q2));
            D = C-A;
            E = 2*B;
            F = A+C;
            if this.LR == 0 
                this.q3 = 2*atan((-E+sqrt(E^2-4*D*F))/(2*D));
                q3_ = this.q3 - pi; %q3 at P3
                phi_ = this.q3 - phi;
            else
                this.q3 = 2*atan((-E-sqrt(E^2-4*D*F))/(2*D));
                q3_ = this.q3 + pi;
                phi_ = this.q3 + phi;
            end
            this.P4 = L3 * [cos(q3_) sin(q3_)] + this.P3;
            this.L4_check = sqrt((this.P4(1)-this.P5(1))^2 + (this.P4(2)-this.P5(2))^2);
            %disp(this.L4_check);

            %compute fingertip coord
            this.ft = f1 * [cos(phi_) sin(phi_)] + this.P4;
            this.f2_check = sqrt((this.ft(1)-this.P3(1))^2 + (this.ft(2)-this.P3(2))^2);
            %disp(this.f2_check);
        end

        function redraw(this)
            this.setLink(this.link1,this.P5,this.P1);
            this.setLink(this.link2,this.P1,this.P2);
            this.setLink(this.link3,this.P2,this.P3);
            this.setLink(this.link4,this.P3,this.P4);
            this.setLink(this.link5,this.P4,this.P5);
            this.setLink(this.link6,this.P4,this.ft);
            this.setLink(this.link7,this.ft,this.P3);
            set(this.joint1,'XData',this.P1(1),'YData',this.P1(2));
            set(this.joint2,'XData',this.P2(1),'YData',this.P2(2));
            set(this.joint3,'XData',this.P3(1),'YData',this.P3(2));
            set(this.joint4,'XData',this.P4(1),'YData',this.P4(2));
            set(this.joint5,'XData',this.P5(1),'YData',this.P5(2));
            if this.LR == 0
                color = {'#0072BD','#D95319','#EDB120','#7E2F8E','#77AC30','#4DBEEE','#A2142F'};
                plot(this.graph,this.ft(1),this.ft(2), '.','Color',color{mod(round(rad2deg(this.Q2)),length(color))+1});
            else
                plot(this.graph,this.ft(1),this.ft(2),'.r');
            end
        end

    end

    methods(Static)
        function link = drawLink(p1,p2,style)
            link = plot([p1(1) p2(1)],[p1(2) p2(2)],style);
        end

        function setLink(link,p1,p2)
            x = [p1(1) p2(1)];
            y = [p1(2) p2(2)];
            set(link,'XData', x, 'YData',y);
        end

    end

end