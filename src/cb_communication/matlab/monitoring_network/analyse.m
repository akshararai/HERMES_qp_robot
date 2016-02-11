log_monitoring = load('../../build_rt_thread_tester/log_monitoring');
log_monitoring = log_monitoring(2:end,:);

%old_stuff = load('monitor_exp2.dat');

%t_old = (1:length(old_stuff(:,1)))/500;


initTime = log_monitoring(1,2);
tTime = (log_monitoring(:,2)-initTime)*10^9 + log_monitoring(:,1);

mainLoop = diff(tTime)/1000;

log_monitoring = log_monitoring(2:end,:);

%mainLoop = log_monitoring(:,3);
t = (1:length(mainLoop))/500;


m_mean = mean(mainLoop);
m_std = std(mainLoop);
m_median = median(mainLoop);
m_iqr = iqr(mainLoop);

p_2100 = length(find(mainLoop>2100))/length(mainLoop)*100;
p_3000 = length(find(mainLoop>3000))/length(mainLoop)*100;
p_4000 = length(find(mainLoop>4000))/length(mainLoop)*100;
p_6000 = length(find(mainLoop>6000))/length(mainLoop)*100;

m_max = max(mainLoop);
m_min = min(mainLoop);

fprintf('mean %f, median %f, std %f, iqr %f\n\n\n',m_mean,m_median,m_std,m_iqr);
fprintf('>2100: %f %%, >3000: %f %% >4000: %f %% >6000: %f %%\n\n\n',p_2100,p_3000,p_4000,p_6000);
fprintf('min Time: %f, max Time %f\n\n\n',m_min,m_max);

%%xplot(t,log_monitoring(:,7),t,log_monitoring(:,9),t,log_monitoring(: ...
           %                                       ,11),t,log_monitoring(:,13));
figure;
%subplot(2,1,1);
%set(gca,'fontsize',20);
%plot(t_old,old_stuff(:,3),'linewidth',2);
%ylabel('Cycle [us]');
%ylim([0 20000]);
%text(20,18000,'BEFORE - NO RTnet','fontsize',30);

%subplot(2,1,2);
plot(t,mainLoop,'linewidth',2);
set(gca,'fontsize',20);
ylabel('Cycle [us]');
xlabel('Time [s]');
%ylim([0 20000]);
%text(20,18000,'AFTER - RTnet ON - Gyroscope and SL running','fontsize',30);
%text(20,16000,'Minimum send-receive loop 1744us, maximum 2329us, mean 1764us, std 17','fontsize',20);
%text(20,12000,'Packet Loss statistics:','fontsize',20);
%text(20,10000,'Net2=0.5%, Net3=0.0006%, Net4=0.004% and Net 5= 0.2%','fontsize',20);
%text(20,8000,'When we have packet loss in general we lose 2 packets for a network','fontsize',20);

numLoop = length(t)-1;

sendLoss = log_monitoring(:,5:3:end);
receiveLoss = log_monitoring(:,6:3:end);

numSendLoss = sum(sendLoss,1)/numLoop*100;
numReceiveLoss = sum(receiveLoss,1)/numLoop*100;


for i = 1:5
  tmp = find(receiveLoss(:,i)>0);
  min(receiveLoss(tmp,i))
  numFail(i) = length(tmp)/numLoop*100;
end

fprintf('send loss net 1: %d net 2: %d, net 3: %d, net 4: %d, net 5: %d\n',numSendLoss(1),numSendLoss(2),numSendLoss(3),numSendLoss(4),numSendLoss(5));

fprintf('receive loss net 1: %d net 2: %d, net 3: %d, net 4: %d, net 5: %d\n',numReceiveLoss(1),numReceiveLoss(2),numReceiveLoss(3),numReceiveLoss(4),numReceiveLoss(5));

fprintf('receive loss net 1: %d, net 2: %d, net 3: %d, net 4: %d, net 5: %d\n',numFail(1),numFail(2),numFail(3),numFail(4),numFail(5));

figure
for i = 1:5
  subplot(5,1,i);
  plot(t,sendLoss(:,i),t,receiveLoss(:,i));
end