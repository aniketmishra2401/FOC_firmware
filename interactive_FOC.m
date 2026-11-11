function interactive_FOC

P.Vdc=12; P.Rs=0.18; P.Ld=100e-6; P.Lq=100e-6; P.polePairs=7;
P.Kv=1000; P.lambdaPM=60/(2*pi*P.Kv)/(1.5*P.polePairs);
P.J=2e-5; P.B=1e-5; P.Ts=50e-6; P.Imax=6;
P.Kt=1.5*P.polePairs*P.lambdaPM; P.Tmax=P.Kt*P.Imax;
P.KpI=P.Ld*2*pi*400; P.KiI=P.Rs*2*pi*400;
P.KpPos=25; P.KdPos=10;

x.id=0; x.iq=0; x.w=0; x.theta=0; x.pos=0; x.intD=0; x.intQ=0;
loadNm=0.005; command=0.006; mode='Torque'; sp =[]; rxBuffer=''; runTimer=[]; isRunning=false;

f=figure('Name','Interactive A2212 FOC simulation','NumberTitle','off', ...
    'Color','w','Position',[100 80 1150 780],'CloseRequestFcn',@closeApp);
ax1=subplot(3,2,1,'Parent',f); hSpeed=animatedline(ax1,'Color',[0 .35 .8],'LineWidth',1.3);
title(ax1,'Motor speed'); xlabel(ax1,'Time (s)'); ylabel(ax1,'rpm'); grid(ax1,'on');
ax2=subplot(3,2,2,'Parent',f); hIa=animatedline(ax2,'Color',[.85 .25 .2],'LineWidth',1);
hIb=animatedline(ax2,'Color',[.2 .55 .25],'LineWidth',1);
hIc=animatedline(ax2,'Color',[.25 .35 .8],'LineWidth',1);
title(ax2,'Simulated phase currents'); xlabel(ax2,'Time (s)'); ylabel(ax2,'A'); grid(ax2,'on');
legend(ax2,{'i_a','i_b','i_c'});
ax3=subplot(3,2,3,'Parent',f); hD=animatedline(ax3,'Color',[.2 .5 .8]);
hQ=animatedline(ax3,'Color',[.85 .3 .2]); hQref=animatedline(ax3,'Color',[.1 .1 .1],'LineStyle','--');
title(ax3,'dq currents and q-current command'); xlabel(ax3,'Time (s)'); ylabel(ax3,'A'); grid(ax3,'on');
legend(ax3,{'i_d','i_q','i_q^*'});
ax4=subplot(3,2,4,'Parent',f); hTorque=animatedline(ax4,'Color',[.85 .3 .15],'LineWidth',1.2);
hTorqueRef=animatedline(ax4,'Color',[.1 .1 .1],'LineStyle','--');
title(ax4,'Electromagnetic torque'); xlabel(ax4,'Time (s)'); ylabel(ax4,'N m'); grid(ax4,'on');
legend(ax4,{'actual','command'});
ax5=subplot(3,2,5,'Parent',f); hPos=animatedline(ax5,'Color',[.15 .5 .25],'LineWidth',1.2);
hPosRef=animatedline(ax5,'Color',[.1 .1 .1],'LineStyle','--');
title(ax5,'Mechanical position (position mode)'); xlabel(ax5,'Time (s)'); ylabel(ax5,'degrees'); grid(ax5,'on');
legend(ax5,{'actual','target'});
ax6=subplot(3,2,6,'Parent',f); hold(ax6,'on'); axis(ax6,'equal');
rectangle(ax6,'Position',[-1 -1 2 2],'Curvature',[1 1],'EdgeColor',[.25 .25 .25],'LineWidth',2);
for poleAngle=0:60:300
    plot(ax6,[.72*cosd(poleAngle) .92*cosd(poleAngle)], ...
        [.72*sind(poleAngle) .92*sind(poleAngle)],'Color',[.65 .65 .65],'LineWidth',5);
end
rectangle(ax6,'Position',[-.48 -.48 .96 .96],'Curvature',[1 1], ...
    'FaceColor',[.92 .94 .97],'EdgeColor',[.35 .35 .4],'LineWidth',1.5);
hTarget=plot(ax6,[0 .88],[0 0],'k--','LineWidth',1.2);
hRotorN=plot(ax6,[0 .76],[0 0],'Color',[.85 .2 .18],'LineWidth',7);
hRotorS=plot(ax6,[0 -.76],[0 0],'Color',[.15 .35 .85],'LineWidth',7);
hLabelN=text(ax6,.57,0,'N','Color','w','FontWeight','bold','HorizontalAlignment','center');
hLabelS=text(ax6,-.57,0,'S','Color','w','FontWeight','bold','HorizontalAlignment','center');
xlim(ax6,[-1.15 1.15]); ylim(ax6,[-1.15 1.15]); axis(ax6,'off');
title(ax6,'Rotor angle visualization');

% Explicit light styling avoids low-contrast labels under MATLAB dark themes.
plotAxes=[ax1 ax2 ax3 ax4 ax5 ax6];
for ax=plotAxes
    set(ax,'Color','w','XColor',[.12 .12 .12],'YColor',[.12 .12 .12], ...
        'GridColor',[.72 .72 .72],'FontSize',10,'LineWidth',1);
    set(get(ax,'Title'),'Color',[.08 .08 .08],'FontWeight','bold');
    set(get(ax,'XLabel'),'Color',[.08 .08 .08]);
    set(get(ax,'YLabel'),'Color',[.08 .08 .08]);
end
set(ax1,'Position',[.08 .70 .39 .23]);
set(ax2,'Position',[.56 .70 .39 .23]);
set(ax3,'Position',[.08 .43 .39 .23]);
set(ax4,'Position',[.56 .43 .39 .23]);
set(ax5,'Position',[.08 .17 .39 .22]);
set(ax6,'Position',[.56 .17 .39 .22]);
legends=findall(f,'Type','legend');
for lg=legends'
    set(lg,'Color','w','TextColor',[.08 .08 .08],'EdgeColor',[.4 .4 .4]);
end

uicontrol(f,'Style','pushbutton','String','Start / Pause','Units','normalized', ...
    'Position',[.02 .015 .10 .04],'Callback',@toggleRun);
uicontrol(f,'Style','text','String','Mode','Units','normalized', ...
    'Position',[.13 .015 .05 .035],'BackgroundColor','w','HorizontalAlignment','left');
uicontrol(f,'Style','popupmenu','String',{'Torque','Position'},'Value',1, ...
    'Units','normalized','Position',[.18 .015 .13 .04],'Callback',@modeChanged);
commandLabel=uicontrol(f,'Style','text','String','','Units','normalized', ...
    'Position',[.33 .015 .20 .035],'BackgroundColor','w','HorizontalAlignment','left');
sCommand=uicontrol(f,'Style','slider','Min',-P.Tmax,'Max',P.Tmax,'Value',command, ...
    'Units','normalized','Position',[.52 .02 .30 .03],'Callback',@commandChanged);
tCommand=uicontrol(f,'Style','text','String','','Units','normalized','Position',[.83 .015 .08 .035],'BackgroundColor','w');
uicontrol(f,'Style','text','String','Load (N m)','Units','normalized', ...
    'Position',[.02 .065 .11 .035],'BackgroundColor','w','HorizontalAlignment','left');
sLoad=uicontrol(f,'Style','slider','Min',0,'Max',0.04,'Value',loadNm, ...
    'Units','normalized','Position',[.13 .07 .23 .03],'Callback',@loadChanged);
tLoad=uicontrol(f,'Style','text','String','','Units','normalized','Position',[.37 .065 .07 .035],'BackgroundColor','w');
uicontrol(f,'Style','text','String','ESP32 port:','Units','normalized', ...
    'Position',[.48 .065 .09 .035],'BackgroundColor','w','HorizontalAlignment','left');
ePort=uicontrol(f,'Style','edit','String','COM5','Units','normalized', ...
    'Position',[.57 .065 .12 .04],'BackgroundColor','w');
uicontrol(f,'Style','pushbutton','String','Connect ESP32','Units','normalized', ...
    'Position',[.70 .065 .12 .04],'Callback',@connectESP);
hStatus=uicontrol(f,'Style','text','String','Sliders active; connect ESP32 for two-knob input.', ...
    'Units','normalized','Position',[.83 .065 .16 .04],'BackgroundColor','w','HorizontalAlignment','left');
textControls=findall(f,'Style','text');
set(textControls,'ForegroundColor',[.08 .08 .08]);
set(ePort,'ForegroundColor',[.08 .08 .08],'BackgroundColor','w');
refreshLabels();

    function loadChanged(src,~)
        loadNm=get(src,'Value'); refreshLabels();
    end
    function commandChanged(src,~)
        command=get(src,'Value'); refreshLabels();
    end
    function modeChanged(src,~)
        if get(src,'Value')==1
            mode='Torque'; set(sCommand,'Min',-P.Tmax,'Max',P.Tmax,'Value',max(-P.Tmax,min(P.Tmax,command)));
            command=get(sCommand,'Value');
        else
            mode='Position'; set(sCommand,'Min',-180,'Max',180,'Value',0); command=0;
        end
        refreshLabels();
    end
    function refreshLabels()
        set(tLoad,'String',sprintf('%.3f',loadNm));
        if strcmp(mode,'Torque')
            set(commandLabel,'String','Torque command (N m)'); set(tCommand,'String',sprintf('%.3f',command));
        else
            set(commandLabel,'String','Position target (deg)'); set(tCommand,'String',sprintf('%.1f',command));
        end
    end
    function toggleRun(~,~)
        if isempty(runTimer) || ~isvalid(runTimer)
            runTimer=timer('ExecutionMode','fixedSpacing','Period',0.02,'TimerFcn',@tick);
        end
        if isRunning
            stop(runTimer); isRunning=false; set(hStatus,'String','Paused. Adjust sliders or connect ESP32.');
        else
            start(runTimer); isRunning=true; set(hStatus,'String','Running averaged FOC simulation.');
        end
    end
    function connectESP(~,~)
        try
            if ~isempty(sp), clear sp; end
            sp=serialport(strtrim(get(ePort,'String')),115200,'Timeout',0.02);
            configureTerminator(sp,'LF'); flush(sp);
            rxBuffer='';
            set(hStatus,'String','Connected: CMD,load_Nm,command');
        catch err
            sp=[];
            set(hStatus,'String',['Serial unavailable: ' err.message]);
        end
    end
    function tick(~,~)
        readCommands();
        nSteps=round(0.02/P.Ts);
        torqueCommand=0; iqCmd=0;
        for ii=1:nSteps
            idRef=0;
            if strcmp(mode,'Torque')
                torqueCommand=command;
                iqCmd=max(-P.Imax,min(P.Imax,torqueCommand/P.Kt));
            else
                posError=command*pi/180-x.pos;
                % Critically damped mechanical position command, plus opposing
                % load/friction feed-forward. Inner dq current PI tracks iqCmd.
                accelRef=P.KpPos*posError-P.KdPos*x.w;
                loadDirection=sign(x.w);
                if abs(x.w)<0.1, loadDirection=sign(posError); end
                torqueRaw=P.J*accelRef+P.B*x.w+loadNm*loadDirection;
                torqueCommand=max(-P.Tmax,min(P.Tmax,torqueRaw));
                iqCmd=torqueCommand/P.Kt;
            end
            ed=idRef-x.id; eq=iqCmd-x.iq; we=P.polePairs*x.w;
            vd0=P.KpI*ed+x.intD-we*P.Lq*x.iq;
            vq0=P.KpI*eq+x.intQ+we*(P.Ld*x.id+P.lambdaPM);
            vmax=P.Vdc/sqrt(3); mag=hypot(vd0,vq0); sc=min(1,vmax/max(mag,1e-12));
            vd=vd0*sc; vq=vq0*sc;
            if sc>0.999 || vd0*ed<0, x.intD=x.intD+P.KiI*ed*P.Ts; end
            if sc>0.999 || vq0*eq<0, x.intQ=x.intQ+P.KiI*eq*P.Ts; end
            did=(vd-P.Rs*x.id+we*P.Lq*x.iq)/P.Ld;
            diq=(vq-P.Rs*x.iq-we*(P.Ld*x.id+P.lambdaPM))/P.Lq;
            te=1.5*P.polePairs*P.lambdaPM*x.iq;
            x.id=x.id+P.Ts*did; x.iq=x.iq+P.Ts*diq;
            motionSign=sign(x.w);
            if abs(x.w)<0.2, motionSign=sign(te); end
            if abs(x.w)<0.2 && abs(te)<=loadNm
                x.w=0; % static friction holds the rotor at rest below breakaway torque
                acceleration=0;
            else
                acceleration=(te-loadNm*motionSign-P.B*x.w)/P.J;
            end
            x.w=x.w+P.Ts*acceleration;
            x.pos=x.pos+P.Ts*x.w; x.theta=mod(P.polePairs*x.pos,2*pi);
        end
        tNow=getappdata(f,'simTime'); if isempty(tNow), tNow=0; end
        tNow=tNow+nSteps*P.Ts; setappdata(f,'simTime',tNow);
        a=x.id*cos(x.theta)-x.iq*sin(x.theta);
        b=-.5*a+sqrt(3)/2*(x.id*sin(x.theta)+x.iq*cos(x.theta));
        c=-a-b;
        addpoints(hSpeed,tNow,x.w*60/(2*pi));
        addpoints(hIa,tNow,a); addpoints(hIb,tNow,b); addpoints(hIc,tNow,c);
        addpoints(hD,tNow,x.id); addpoints(hQ,tNow,x.iq); addpoints(hQref,tNow,iqCmd);
        addpoints(hTorque,tNow,te); addpoints(hTorqueRef,tNow,torqueCommand);
        addpoints(hPos,tNow,x.pos*180/pi); addpoints(hPosRef,tNow,command);
        rotorAngle=mod(x.pos,2*pi); targetAngle=command*pi/180;
        set(hRotorN,'XData',[0 .76*cos(rotorAngle)],'YData',[0 .76*sin(rotorAngle)]);
        set(hRotorS,'XData',[0 -.76*cos(rotorAngle)],'YData',[0 -.76*sin(rotorAngle)]);
        set(hLabelN,'Position',[.57*cos(rotorAngle) .57*sin(rotorAngle) 0]);
        set(hLabelS,'Position',[-.57*cos(rotorAngle) -.57*sin(rotorAngle) 0]);
        set(hTarget,'XData',[0 .88*cos(targetAngle)],'YData',[0 .88*sin(targetAngle)]);
        if strcmp(mode,'Position'), set(hTarget,'Visible','on'); else, set(hTarget,'Visible','off'); end
        if strcmp(mode,'Torque') && abs(x.w)<1 && abs(command)<=loadNm
            set(hStatus,'String','Stalled: torque command is below load. Increase torque or reduce load.');
        elseif strcmp(mode,'Torque')
            set(hStatus,'String','Running in torque mode.');
        elseif strcmp(mode,'Position') && loadNm>0.6*P.Tmax
            set(hStatus,'String','Load is near the torque limit; position corrections will be weak.');
        else
            set(hStatus,'String','Running in position mode.');
        end
        drawnow limitrate;
    end
    function readCommands()
        if isempty(sp), return; end
        try
            if sp.NumBytesAvailable>0
                rxBuffer=[rxBuffer char(read(sp,sp.NumBytesAvailable,'char'))];
            end
            newlineAt=find(rxBuffer==char(10),1,'last');
            while ~isempty(newlineAt)
                line=strtrim(rxBuffer(1:newlineAt-1));
                rxBuffer=rxBuffer(newlineAt+1:end);
                vals=sscanf(line,'CMD,%f,%f');
                if numel(vals)==2 && all(isfinite(vals))
                    loadNm=max(0,min(0.04,vals(1)));
                    fraction=max(0,min(1,vals(2)));
                    if strcmp(mode,'Torque'), command=(2*fraction-1)*P.Tmax;
                    else, command=(2*fraction-1)*180; end
                    set(sLoad,'Value',loadNm); set(sCommand,'Value',command); refreshLabels();
                end
                newlineAt=find(rxBuffer==char(10),1,'last');
            end
        catch
            set(hStatus,'String','Serial read error; sliders remain available.');
            sp=[];
        end
    end
    function closeApp(~,~)
        try, if ~isempty(runTimer) && isvalid(runTimer), stop(runTimer); delete(runTimer); end, catch, end
        try, if ~isempty(sp), clear sp; end, catch, end
        delete(f);
    end
end
