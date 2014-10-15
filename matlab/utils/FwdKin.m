% Edited by Alessandro Roncone
% Genova Oct 2013
function [chain] = FwdKin(body_part)
% This function computes and displays the forward kinematics of a body part
% INPUT
%   body_part (struct) - the body part under consideration. Each body part has the same structure:
%     name = the name of the body_part;
%     H0   = is the roto-translation matrix in the origin of the chain (if the body part is attached
%            to another one, tipically the last reference frame of the previous body part goes here)
%     H_0  = is the roto-translation matrix used internally for some special stuff (for example in
%            fingers). It's attached after H0 and it shouldn't be modified by the user. Most of the
%            time, it can be an identity matrix.
%     DH   = it's the parameter matrix. Each row has 4 DH parameters (a, d, alpha, offset), thus each
%            row completely describes a link. The more rows are added, the more links are attached.
%     Th   = it's the joint values vector (as read from the encoders)
%  Please note that everything is in SI units (the eventual conversions will be handled by
%  the algorithm itself).
%
% OUTPUT
%   chain (struct) - the resulted chain with everything inside it. It's divided by body parts.

    %% MISC STUFF
        ljnt  = 7;               % joint pic length
        rjnt  = 3;               % joint pic radius
        linkratio = 1/15;        % link dimension ratio

        LinkColor  = body_part.LinkColor;
        linkTransparency = 0.6;

        JntColor   = [.7 .7 .7];  % RGB color of the joints

        scaling = body_part.scalingfactor;

    %% PARAMETERS

        H0  = body_part.H0();
        H0(1:3,4)  = H0(1:3,4).*1000;

        H_0 = body_part.H_0();
        H_0(1:3,4) = H_0(1:3,4).*1000;

        DH  = body_part.DH;
        DH(:,1:2)  = DH(:,1:2).*1000;

        theta = body_part.Th';

        %%% TOTAL D-H;
        a    = DH(:,1);
        d    = DH(:,2);
        alph = DH(:,3);
        offs = DH(:,4);

        %%% THETAS!
        thet = theta + offs;

        %%% CHAIN
        RTMat    = cell(1,length(a)+1);
        RFFrame  = cell(1,length(a)+1+1);
        cyl      = cell(1,length(a)+1);

        RFFrame{1} = H0;
        RFFrame{2} = H0*H_0;
        RTMat{1}   = H_0;

        for i = 1:length(a)
            % i-th link roto-translation
            RTMat{i+1} = evalDHMatrix ( a(i), d(i), alph(i), thet(i));
            % from root to i-th link
            RFFrame{i+2} = RFFrame{i+1} * RTMat{i+1};
        end

        % Draw the stuff (joints, ref frames, links)
        for i = 1:length(RFFrame)-2
            drawCylinder(ljnt, rjnt, RFFrame{i+1} * [1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.8);
        end
        drawSphere(rjnt-1,RFFrame{end}(1:3,4), JntColor, 100, 0.9);

        if isfield(body_part,'HN')
            RTMat{end+1} = body_part.HN;
            RFFrame{end+1} = RFFrame{end} * RTMat{end};
            drawSphere(rjnt-1,RFFrame{end}(1:3,4), JntColor, 100, 0.9);
        end

        drawRefFrame(RFFrame{1},scaling,1,'hat');
        drawRefFrame(RFFrame{end},scaling,i);

        for i = 1:length(RFFrame)-1
            cyl{i} = drawCylinderFromTo(RFFrame{i}(1:3,4),RFFrame{i+1}(1:3,4), LinkColor, 100, linkTransparency, linkratio);
        end

    %%%%%%%%%%%%%%
    % FINAL STUFF - CREATE THE OUTPUT DATA FILE (I.E. CHAIN)
    %%%%%%%%%%%%%%
        chain.name = body_part.name;
        chain.H0   = body_part.H0;
        chain.H_0  = body_part.H_0;
        chain.DH   = body_part.DH;
        chain.Th   = body_part.Th;

        chain.a    = a;
        chain.d    = d;
        chain.alph = alph;
        chain.offs = offs;

        chain.RTMat   = RTMat;
        chain.RFFrame = RFFrame;

        chain.LinkColor = LinkColor;
end