clear; clc;
s = serialport("COM9", 250000);

% dist, AoA 배열 초기화
dist_array = zeros(1, 800);
AoA_array = zeros(1, 800);

arrayIndex = 1; 

% 그림 및 서브플롯 초기화
figure(1);

subplot(1, 1, 1); % 실시간 좌표
% title('실시간 좌표');
title('Moving Average');
xlim([-1 1]);
ylim([0 2]);
axis normal;
axis square;


while true
    str = string(read(s, 30, "char")); 
    str1 = extractBetween(str, "D", "A");

    if isempty(str1)
        flushinput(s);
        continue;
    else
        if arrayIndex > 800 % 리스트 다 차면 초기화
            dist_array = zeros(1, 800);
            AoA_array = zeros(1, 800);
            arrayIndex = 1;
        end

        dist = str2double(extractBetween(str1, "I", "P"));
        AoA = str2double(extractBetween(str1, "P", "O")); % "P"와 "O" 사이가 아닌 "P"와 "E" 사이로 수정

%         x1 = dist * sin(deg2rad(AoA));
%         y1 = dist * cos(deg2rad(AoA));

%  실시간 좌표 업데이트
%         subplot(1, 1, 1);
%         hold on;
%         %rectangle('Position',[-0.9 2.7 1.8 1.8],'EdgeColor','r','LineWidth',2); % 직사각형 추가
%         scatter(x1, y1, 'o', 'filled');
%         plot(0, 0, 'r*');
%         text(-0.25, 0, 'Anchor');

        dist_array(arrayIndex) = dist; % 좌표 표시 리스트
        AoA_array(arrayIndex) = AoA;

        arrayIndex = arrayIndex + 1; % 인덱스 업데이트


        num = 7;
        if mod(arrayIndex, 8) == 0 % 8개의 값마다 평균, 중앙값, 최빈값을 출력
            part_list_dist = dist_array(arrayIndex-num : arrayIndex);
            part_list_AoA = AoA_array(arrayIndex-num : arrayIndex);
    
            valid_data_dist = part_list_dist(~isnan(part_list_dist));  % nan값 빼고 추출하기
            valid_data_AoA = part_list_AoA(~isnan(part_list_AoA));
            disp(length(valid_data_AoA));

            avgdist = mean(valid_data_dist);
            avgAoA = mean(valid_data_AoA);
    
            x2 = avgdist * sin(deg2rad(avgAoA));        % 평균값을 이용
            y2 = avgdist *cos(deg2rad(avgAoA));
    
           
            subplot(1, 1, 1);
            hold on;
%             rectangle('Position',[-0.9 2.7 1.8 1.8],'EdgeColor','r','LineWidth',2); % 직사각형 추가
            scatter(x2,y2,'o','filled');
            plot(0,0,'r*');
            text(-0.25, 0, 'Anchor');
        end

        flushinput(s);
    end
end