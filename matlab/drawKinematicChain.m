function [chain] =drawKinematicChain(filename,workspacelimits)
%DRAWKINEMATICCHAIN Imports a kinematic chain and draws it on the figure
%
% Example:
%   chain =drawKinematicChain(filename,workspacelimits);
%
    M_PI = pi;
    CTRL_DEG2RAD = pi/180;

%% Read H0
    tmpl='H0';
    s=textread(filename,'%s','delimiter','\n','whitespace','');
    ix=strncmp(s,tmpl,numel(tmpl));
    s=s(ix);
    s=strrep(s,'H0 ','');
    s=strrep(s,'(','');
    s=strrep(s,')','');
    
    if sum(ix)~=0
        ch.H0 = str2num(s{1});
        ch.H0 = reshape(ch.H0,[4 4])';
    else
        ch.H0 = eye(4);
    end
    disp('H0:');
    disp(ch.H0);

%% Read HN
    tmpl='HN';
    s=textread(filename,'%s','delimiter','\n','whitespace','');
    ix=strncmp(s,tmpl,numel(tmpl));
    s=s(ix);
    s=strrep(s,'HN ','');
    s=strrep(s,'(','');
    s=strrep(s,')','');
    
    if sum(ix)~=0
        ch.HN = str2num(s{1});
        ch.HN = reshape(ch.H0,[4 4]);
    else
        ch.HN = eye(4);
    end
    disp('HN:')
    disp(ch.HN);

%% Read Links
    clear tmpl s;
    tmpl='numLinks';
    s=textread(filename,'%s','delimiter','\n','whitespace','');
    ix=strncmp(s,tmpl,numel(tmpl));
    s=s(ix);
    s=strrep(s,'numLinks ','');
    s=strrep(s,' ','');
    
    if sum(ix)~=0
        ch.numLinks=str2num(s{1});
    else
        disp('ERROR! no numLinks has been found!');
    end
    disp(sprintf('numLinks: %i',ch.numLinks));

    DHmat=[];
    for i=1:ch.numLinks
        tmpl=strcat('link_',num2str(i-1));
        s=textread(filename,'%s','delimiter','\n','whitespace','');
        ix=strncmp(s,tmpl,numel(tmpl));
        s=s(ix);
        delimiter = {'numLinks','offset','alpha','link_','min','max','H0',' ','(',')','A','D','home'};
        formatSpec = '%f%f%f%f%f%f%f%[^\n\r]';
        ts=textscan(s{1},formatSpec,'Delimiter',delimiter,'MultipleDelimsAsOne',true);
        ts=ts(1:end-1);
        disp(tmpl)
        disp(ts);
        DHmat=[DHmat; cell2mat(ts)];
    end
    DHmat=DHmat(:,2:end);
    ch.DH=DHmat;
    ch.DH(:,3:4)=ch.DH(:,3:4).*CTRL_DEG2RAD;

%% Read home
    clear tmpl s
    tmpl='home';
    s=textread(filename,'%s','delimiter','\n','whitespace','');
    ix=strncmp(s,tmpl,numel(tmpl));
    s=s(ix);
    s=strrep(s,'home','');
    s=strrep(s,'(','');
    s=strrep(s,')','');
    
    if sum(ix)~=0
        ch.Th = str2num(s{1});
    else
        ch.Th = (ch.DH(:,end)+ch.DH(:,end-1))/2;
        ch.Th = ch.Th';
    end

    if length(ch.Th)~=ch.numLinks
        disp('ERROR! The home configuration has less links that the needed number!');
        disp('Setting home to default');
        ch.Th = (ch.DH(:,end)+ch.DH(:,end-1))/2;
        ch.Th = ch.Th';
    end

    disp('home:');
    disp(ch.Th);


    [o,n,e]=fileparts('../app/conf/DH_right.ini');
    ch.name = n(4:end);

%     ch.DH = [rawmat(:,1:4)];
%     ch.DH(:,3:4)=ch.DH(:,3:4).*CTRL_DEG2RAD;

%     ch.Th = rraw(3,1:rraw(2,1));
%     ch.Th = ch.Th * CTRL_DEG2RAD;
    ch.LinkColor = rand(1,3);
    ch.H_0 = eye(4);

    workspacelimits=workspacelimits/200;
    ch.scalingfactor = workspacelimits;

    chain=FwdKin(ch);

end