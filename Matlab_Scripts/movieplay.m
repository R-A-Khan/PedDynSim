clear all; close all;


for case_num = [11]
    str_case = sprintf('results/case_%02d.mat', case_num); 
    sprintf('Initiating simulation video for case %02d.\n', case_num)

    load(str_case)
    %%
    % movie speed, 1=30 fps
    step=10;
    % Assign single colour to pedestrians based on corridor AB or CD
    if case_num <=4
        cor_color = true;
    else
        cor_color = false;
    end



    color=[];
    cir=0:0.01:2*pi;
    if N==n_groups(1) % if there are no groups random colors
        color=[rand(N,1) rand(N,1) rand(N,1)];
    elseif cor_color
            for i = 1:2
                color=[color; ones(n_groups(i),1)*rand ones(n_groups(i),1)*rand ones(n_groups(i),1)*rand];
            end
    else %if there are groups a color for each group
        for i=1:length(n_groups)
            color=[color; ones(n_groups(i),1)*rand ones(n_groups(i),1)*rand ones(n_groups(i),1)*rand];
        end
    end

    str_mov = sprintf('simulation_%02d',case_num);

    %%% MOVIE
    scrsz = get(0,'ScreenSize');
    vid=VideoWriter(str_mov);
    open(vid);
    h=figure('Color','w','Position',[1 1 scrsz(3) scrsz(4)]);
    % figure(1);
    for tt=1:step:length(tspan)
        % Plot of the walls
        for i=1:num_walls
            plot(map_walls(2*i-1,:),map_walls(2*i,:),'k','LineWidth',2);
            axis equal
            axis([0 25 0 25]);
            hold on
        end

        % Plot of the pedestrians represented as circles
        if cor_color
                for i=1:N/2
        %         axis tight manual;
                plot(r(i)*cos(cir)+X(tt,6*i-5),r(i)*sin(cir)+X(tt,6*i-4),'Color',color(1,:),'LineWidth',2) % plot cerchi
                plot(r(i)*cos(X(tt,6*i-3))+X(tt,6*i-5),r(i)*sin(X(tt,6*i-3))+X(tt,6*i-4),'ok')
        %         axis equal
                axis([0 25 0 25]);
                str = sprintf('HSFM Simulation t=%0.2f s',(tt-1)*(1/30));
                title(str, 'Interpreter','Latex','FontSize',20);
                end
                for i=N/2+1:N
        %         axis tight manual;
                plot(r(i)*cos(cir)+X(tt,6*i-5),r(i)*sin(cir)+X(tt,6*i-4),'Color',color(2,:),'LineWidth',2) % plot cerchi
                plot(r(i)*cos(X(tt,6*i-3))+X(tt,6*i-5),r(i)*sin(X(tt,6*i-3))+X(tt,6*i-4),'ok')
        %         axis equal
                axis([0 25 0 25]);
                str = sprintf('HSFM Simulation t=%0.2f s',(tt-1)*(1/30));
                title(str, 'Interpreter','Latex','FontSize',20);
                end
        else
            for i=1:N
        %         axis tight manual;
                plot(r(i)*cos(cir)+X(tt,6*i-5),r(i)*sin(cir)+X(tt,6*i-4),'Color',color(i,:),'LineWidth',2) % plot cerchi
                plot(r(i)*cos(X(tt,6*i-3))+X(tt,6*i-5),r(i)*sin(X(tt,6*i-3))+X(tt,6*i-4),'ok')
        %         axis equal
                axis([0 25 0 25]);
                str = sprintf('HSFM Simulation t=%0.2f s',(tt-1)*(1/30));
                title(str, 'Interpreter','Latex','FontSize',20);
            end
        end

        hold off

    %     axis off
    %     title('HSFM Simulation','Interpreter','Latex','FontSize',20)
        M=getframe(h);
    %     M=getframe();
        writeVideo(vid,M);
    end
    close(vid)
    sprintf('Simulation video for case %02d completed.\n', case_num)
end
