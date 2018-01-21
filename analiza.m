clear;clc;close all;

data = xlsread('analiza.xlsx', 1);

time = data(:,2);
frame = data(:,1);
marker0 = zeros(185,2,51);

% s(1) = subplot(2,1,1);
% s(2) = subplot(2,1,2);

% pierwszy wymiar to chwila czasowa
% drugi wymiar to kierunek x i y
% trzeci wymiar to numer marker0a

for i=1:51

    marker0(:,:,i) = [data(:,2*i+1), data(:,2*i+2)]; 
     
%     % wykresy zmiennych wejœciowych
% 
%     plot(s(1),frame,marker0(:,1,i))
%     axis(s(1),[0 185 250 1150])
%     title(s(1),['polozenie marker0a ', num2str(i)])
%     ylabel(s(1),'x')
%     xlabel(s(1),'czas')
%     
%     plot(s(2),frame,marker0(:,2,i))
%     axis(s(2),[0 185 0 550])
%     title(s(2),['polozenie marker0a ', num2str(i)])
%     ylabel(s(2),'y')
%     xlabel(s(2),'czas')
%     
%     t=input(['nastêpna klatka ', num2str(i+1)]);
     
end

%% 
% przeliczenie odpowiedniego k¹ta obrotu
data = xlsread('wymiary.xlsx', 1);

p1 = [data(1); data(2)];
p2 = [data(3); data(4)];
p3 = [data(5); data(6)];
p4 = [data(7); data(8)];

fi = -atan2((p1(1)-p2(1)), (p2(2)-p1(2)));
R = [cos(fi), -sin(fi); sin(fi), cos(fi)];

p1 = R * p1;
p2 = R * p2;
p3 = R * p3;
p4 = R * p4;

% linijka

l1 = [data(9); data(10)];
l2 = [data(11); data(12)];
l1 = R * l1;
l2 = R * l2;

l = sqrt((l1(1)-l2(1))^2 + (l1(2)-l2(2))^2)/10; % ile pixeli odpowiada jednemu mm

% obrót wszystich wspó³¿êdnych
for i=1:51
    for j=1:185
        marker0(j,:,i) = (R * (marker0(j,:,i))')';
    end
end

%% 
% przyjêcie pocz¹tku uk³adu wspó³rzêdnych 
% x - po³¹cznie g³owotu³owia i odw³oku dla pierwszej klatki
% y - najwiêksza wspó³rzêdna na markera 22
x0 = marker0(1,1,49);
y0 = marker0(1,2,22);
z0l = p1(1);
z0p = p3(1);

% obliczanie wspó³rzêdnych 'pocz¹tku' nogi 
gto = [data(13); data(14)];
s1 = [data(15); data(16)];
s2 = [data(17); data(18)];
s3 = [data(19); data(20)];
s4 = [data(21); data(22)];
s5 = [data(23); data(24)];
s6 = [data(25); data(26)];
s7 = [data(27); data(28)];
s8 = [data(29); data(30)];

gto = R * gto;
s1 = R * s1;
s2 = R * s2;
s3 = R * s3;
s4 = R * s4;
s5 = R * s5;
s6 = R * s6;
s7 = R * s7;
s8 = R * s8;

z = zeros (8,2);
for k=1:8
    z(k,:) = (R * [data(30+2*k-1); data(30+2*k)])';
end

%UWAGA 'z' bêd¹ ju¿ przeliczone !!

zgto1 = (R * [data(47); data(48)])';
zgto2 = (R * [data(49); data(50)])';
zgto = ((zgto1(1)-z0p)+(z0l-zgto2(1)))/2;

z1 = z(1,1) - z0p;
z2 = z(2,1) - z0p;
z3 = z(3,1) - z0p;
z4 = z(4,1) - z0p;
z5 = z0l - z(5,1);
z6 = z0l - z(6,1);
z7 = z0l - z(7,1);
z8 = z0l - z(8,1);

gto = [gto; zgto];
s1 = [s1; z5];
s2 = [s2; z6];
s3 = [s3; z7];
s4 = [s4; z8];
s5 = [s5; z1];
s6 = [s6; z2];
s7 = [s7; z3];
s8 = [s8; z4];

% obliczenie wektorów które trzeba dodaæ do wspórzêdnych po³oczenia
% g³owotu³owia z odw³okiem aby uzyskaæ wspó³rzêdne 'pocz¹tku' nogi
s1 = (s1 - gto)';
s2 = (s2 - gto)';
s3 = (s3 - gto)';
s4 = (s4 - gto)';
s5 = (s5 - gto)';
s6 = (s6 - gto)';
s7 = (s7 - gto)';
s8 = (s8 - gto)';

sR = zeros(4,3);
sR(1,:) = s1;
sR(2,:) = s2;
sR(3,:) = s3;
sR(4,:) = s4;

sL = zeros(4,3);
sL(1,:) = s5;
sL(2,:) = s6;
sL(3,:) = s7;
sL(4,:) = s8;

%% 
% oliczenie wektorów ze wspó³rzêdnymi dla ka¿dej z nóg
% trzeci wymiar odpowiada numerom nogi
% czwarty wymiar odpowiada kolejnym stawom w nodze

R = zeros(185,3,4,4);
for j=1:185
    for k=1:4
        for i=[1 2 3]
            
            R(j,1,k,i)= -(marker0(j,1,(i+(k-1)*3)) - x0);
            R(j,2,k,i)= -(marker0(j,2,(i+(k-1)*3)) - y0);
            R(j,3,k,i)= z0l - marker0(j,1,(i+36+(k-1)*3));
            
        end
        
        R(j,:,k,4) = [marker0(j,:,49), ((marker0(j,1,50)-z0p)+(z0l-marker0(j,1,51))/2)] + sR(k,:);
        R(j,1,k,4) = -(R(j,1,k,4) - x0);
        R(j,2,k,4) = -(R(j,2,k,4) - y0);
    end
end

L = zeros(185,3,4,4);
for j=1:185
    for k=1:4
        for i=[1 2 3]
            
            L(j,1,k,i)= -(marker0(j,1,i+(k+3)*3) - x0);
            L(j,2,k,i)= -(marker0(j,2,i+(k+3)*3) - y0);
            L(j,3,k,i)= marker0(j,1,(i+12)+(k+3)*3) - z0p;
            
        end
        
        L(j,:,k,4) = [marker0(j,:,49), ((marker0(j,1,50)-z0p)+(z0l-marker0(j,1,51))/2)] + sL(k,:);
        L(j,1,k,4) = -(L(j,1,k,4) - x0);
        L(j,2,k,4) = -(L(j,2,k,4) - y0);
    end
end

COM = zeros(185,3);
for j=1:185
    COM(j,:) = [marker0(j,:,49), ((marker0(j,1,50)-z0p)+(z0l-marker0(j,1,51))/2)];
    COM(j,1) = -(COM(j,1) - x0);
    COM(j,2) = -(COM(j,2) - y0);
end

%zamiana na mm
R = R/l;
L = L/l;
COM = COM/l;

%% 
%filtr Butterwortha
% aby poprawiæ pocz¹tek zaczynam poŸniej animacjê

r = 4; %rz¹d filtru
cof = 5; % czêstotliwoœæ odciêcia w Hz
cof1 = 2; % czêstotliwoœæ odciêcia w Hz dla COM
% dobre wyniki - dla nóg cof = 5, dla COM cof = 2
[b,a] = butter(r,(cof/62.5));
[b1,a1] = butter(r,(cof1/62.5));

% charakterystyki filtrów
% freqz(b,a)
% freqz(b1,a1)

R_old= zeros(185,3,4,4);
L_old = zeros(185,3,4,4);
COM_old = zeros(185,3);


% funkcja filtfilt odpowiada za filtracjê 'do przodu a potem z do ty³u'
for i=1:3
    for j=1:4
        for k=1:4
            zm = filtfilt(b,a,(R(:,i,k,j))');
            R_old(:,i,k,j) = R(:,i,k,j);
            R(:,i,k,j) = zm';
            
            zm = filtfilt(b,a,(L(:,i,k,j))');
            L_old(:,i,k,j) = L(:,i,k,j);
            L(:,i,k,j) = zm';
            
        end
    end
    
    zm = filtfilt(b1,a1,(COM(:,i))');
    COM_old(:,i) = COM(:,i);
    COM(:,i) = zm';
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%% wykresy
% s(1) = subplot(2,3,1);
% s(2) = subplot(2,3,2);
% s(3) = subplot(2,3,3);
% s(4) = subplot(2,3,4);
% s(5) = subplot(2,3,5);
% s(6) = subplot(2,3,6);

%%%%%%%%%%%%%%%%%%%%%%%%%%% dla nóg
% nn = 1; % numer nogi
% ns = 1; % numer stawu
% 
% plot(s(1),R_old(:,1,nn,ns))
% ylabel(s(1),'x')
% xlabel(s(1),'numer klatki')
% 
% plot(s(2),R_old(:,2,nn,ns))
% title(s(2), 'przed filtracja')
% ylabel(s(2),'y')
% xlabel(s(2),'numer klatki')
% 
% plot(s(3),R_old(:,3,nn,ns))
% ylabel(s(3),'z')
% xlabel(s(3),'numer klatki')
% 
% 
% plot(s(4),R(:,1,nn,ns))
% ylabel(s(4),'x')
% xlabel(s(4),'numer klatki')
% 
% plot(s(5),R(:,2,nn,ns))
% title(s(5), 'po filtracji')
% ylabel(s(5),'y')
% xlabel(s(5),'numer klatki')
% 
% plot(s(6),R(:,3,nn,ns))
% ylabel(s(6),'z')
% xlabel(s(6),'numer klatki')

%%%%%%%%%%%%%%%%%%%%%%%%%%% dla COM
% plot(s(1),COM_old(:,1))
% ylabel(s(1),'x')
% xlabel(s(1),'numer klatki')
% 
% plot(s(2),COM_old(:,2))
% title(s(2), 'przed filtracja')
% ylabel(s(2),'y')
% xlabel(s(2),'numer klatki')
% 
% plot(s(3),COM_old(:,3))
% ylabel(s(3),'z')
% xlabel(s(3),'numer klatki')
% 
% 
% plot(s(4),COM(:,1))
% ylabel(s(4),'x')
% xlabel(s(4),'numer klatki')
% 
% plot(s(5),COM(:,2))
% title(s(5), 'po filtracji')
% ylabel(s(5),'y')
% xlabel(s(5),'numer klatki')
% 
% plot(s(6),COM(:,3))
% ylabel(s(6),'z')
% xlabel(s(6),'numer klatki')
% 
% linkaxes([s(1) s(4)],'xy')
% linkaxes([s(2) s(5)],'xy')
% linkaxes([s(3) s(6)],'xy')

%% 
% animacja
% close all;
% h = draw (1, R(:,:,1,:), R(:,:,2,:), R(:,:,3,:), R(:,:,4,:), L(:,:,1,:), L(:,:,2,:), L(:,:,3,:), L(:,:,4,:));
% t=input('rozpoczac animacje?');
% 
% while (1)    
%     for k=(1+1):185
%         update(h,k, R(:,:,1,:), R(:,:,2,:), R(:,:,3,:), R(:,:,4,:), L(:,:,1,:), L(:,:,2,:), L(:,:,3,:), L(:,:,4,:))
%         pause (0.008) % rzeczywistoœci odpowiada 0.008s
%     end
%     
% %     t = input('powtorzyc animacje? [1] [0]');
% %     if t==0
% %         close all;
% %         break;
% %     end
% end

%% 
% % rysowanie trajektorji COM
% figure;
% for j=1:184
%     myline(COM(j,:), COM(j+1,:));
% end
% 
% axis([-70 70 0 180 0 50])
% xlabel('x [mm]'); ylabel('y [mm]'); zlabel('z [mm]') 
% hold off
% grid on

%% 
% obliczenie prêdkoœci
% vel - metod¹ Styrlinga 
% vel_2 - najprostsza œrednia

% % dla po³¹czenia g³owotu³owia z odw³okiem
% v_COM = zeros(185,3);
% for i=1:3
%     [v_COM(:,i), k] = vel_2(185, 1, 0.008, COM(:,i));
% end
% v_COM = v_COM/1000;

% figure;
% p(1) = subplot(1,3,1);
% p(2) = subplot(1,3,2);
% p(3) = subplot(1,3,3);
% 
% plot(p(1),(1+k):(185-k),v_COM((1+k):(185-k),1))
% ylabel(p(1),'vx [m/s]')
% xlabel(p(1),'numer klatki')
% 
% plot(p(2),(1+k):(185-k),v_COM((1+k):(185-k),2))
% ylabel(p(2),'vy [m/s]')
% xlabel(p(2),'numer klatki')
% 
% plot(p(3),(1+k):(185-k),v_COM((1+k):(185-k),3))
% ylabel(p(3),'vz [m/s]')
% xlabel(p(3),'numer klatki')


% % wykres na potrzeby raportu
% v_m = zeros (185);
% for i=1:185
%     v_m(i) = 0.0521;
% end
% 
% figure;
% axes = axes('XTickLabel',{'0','20','40','60','80','100'},'XTick',[11 39 67 95 123 152]);
% axis([11 152 0 0.07]);
% box(axes,'on');
% hold(axes,'all');
% 
% plot(11:152,v_COM(11:152,2))
% plot(11:152,v_m(11:152),'--')
% ylabel('vy [m/s]')
% xlabel('% cyklu')


%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% % dla nóg
% v_R = zeros(185,3,4,4);
% v_L = zeros(185,3,4,4);
% for i=1:3
%     for j=1:4
%         for l=1:4
%             
%             [v_R(:,i,j,l), ~] = vel(185, 1, 0.008, R(:,i,j,l));
%             [v_L(:,i,j,l), k] = vel(185, 1, 0.008, L(:,i,j,l));
%             
%         end
%     end    
% end
% 
% v_R = v_R/1000;
% v_L = v_L/1000;
% 
% figure;
% p(1) = subplot(1,3,1);
% p(2) = subplot(1,3,2);
% p(3) = subplot(1,3,3);
% 
% plot(p(1),(1+k):(185-k),v_R((1+k):(185-k),1,nn,ns))
% ylabel(p(1),'vx [m/s]')
% xlabel(p(1),'numer klatki')
% 
% plot(p(2),(1+k):(185-k),v_R((1+k):(185-k),2,nn,ns))
% ylabel(p(2),'vy [m/s]')
% xlabel(p(2),'numer klatki')
% 
% plot(p(3),(1+k):(185-k),v_R((1+k):(185-k),3,nn,ns))
% ylabel(p(3),'vz [m/s]')
% xlabel(p(3),'numer klatki')

%% 
% wykres kiedy która noga sroi na ziemi
d = 2; %kiedy uznajê ¿e noga ca³y czas jest na ziemi - b³¹d

%sprawdzam tylko staw 1 i zmienn¹ z
g = ones(185,8);
for t=1:185
    for k=1:4
        
        if (R(t,3,k,1)<=d)
            g(t,k)=0;
        end
        if (L(t,3,k,1)<=d)
            g(t,k+4)=0;
        end
        
    end
end

% % wstêpny rysunek kiedy która noga jest na ziemi
% for k=1:8
%     for t=1:184
%         if(g(t,k)==0 && g(t+1,k)==0)
%             plot([t t+1],[k k],'LineWidth',2)
%             hold on
%         end
%     end
% end
% 
% axis([1 185 0.5 8.5])
% xlabel('klatki')
% ylabel('[R1]    [R2]    [R3]    [R4]    [L1]    [L2]    [L3]    [L4]');
% hold off
% grid on


%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% z wykresu odczytano ¿e jeden cykl mieœci siê miêdzy klatkami 11 a 153 
% (od postawienia nogi R1 do jej ponownego postawienia)
a = 11;
b = 152; % to klatka w której nastêpuje ju¿ powtórzenie 

% % odkomentuj aby narysowaæ wykres w skali klatek
% figure;
% axes = axes('YTickLabel',{'R1','R2','R3','R4','L1','L2','L3','L4'},...
%             'XTick',[11 22 25 28 30 48 52 65 82 90 93 98 103 123 125 135 150 152],...
%             'XTickLabel',{'11','22','25','28','30','48','52','65','82','90','93','98','103','123','','135','150', ''});
% 
% % % zale¿noœæ od % cyklu
% % axes = axes('YTickLabel',{'R1','R2','R3','R4','L1','L2','L3','L4'},...
% %             'XTickLabel',{'0','20','40','60','80','100'},'XTick',[11 39 67 95 123 152]);
%         
% box(axes,'on');
% grid(axes,'on');
% hold(axes,'all');
% 
% for k=1:8
%     for t=a:b-1
%         if(g(t,k)==0 && g(t+1,k)==0)
%             plot([t t+1],[k k],'LineWidth',2)
%             hold on
%         end
%     end
% end
% 
% axis([a b 0.5 8.5])
% xlabel('numer klatki')
% hold off
% grid on
 
%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% % odkomentuj aby narysowaæ wykres w skali czasu
% figure;
% axes = axes('YTickLabel',{'R1','R2','R3','R4','L1','L2','L3','L4'},...
%             'XTick',[0.088 0.176 0.2 0.224 0.24  0.384 0.416 0.52 0.656 0.72 0.744 0.784 0.824 0.984 1 1.08 1.2 1.216],...
%             'XTickLabel',{'0.088', '0.176', '0.2', '', '0.24',  '0.384', '0.416', '0.52', '0.656', '0.72', '0.744', '0.784', '0.824', '0.984', '1', '1.08', '1.2', ''});
% box(axes,'on');
% grid(axes,'on');
% hold(axes,'all');
% 
% 
% for k=1:8
%     for t=a:b-1
%         if(g(t,k)==0 && g(t+1,k)==0)
%             ti = [0.008*t 0.008*(t+1)];
%             plot(ti,[k k],'LineWidth',2)
%             hold on
%         end
%     end
% end
% 
% axis([a*0.008 b*0.008 0.5 8.5])
% xlabel('czas [s]')
% hold off
% grid on


%% 
% % wykres kilku cykli z rzêdu
% p = 3;
% figure;
% axes = axes('YTickLabel',{'R1','R2','R3','R4','L1','L2','L3','L4'},...
%             'XTick',[0 141 282],...
%             'XTickLabel',{'cykl 1', 'cykl 2', 'cykl 3'});
% 
% % % zale¿noœæ od % cyklu
% % axes = axes('YTickLabel',{'R1','R2','R3','R4','L1','L2','L3','L4'},...
% %             'XTickLabel',{'0','20','40','60','80','100'},'XTick',[11 39 67 95 123 152]);
%         
% box(axes,'on');
% grid(axes,'on');
% hold(axes,'all');
% 
% for c=0:(p-1)
%     for k=1:8
%         for t=a:b-1
%             if(g(t,k)==0 && g(t+1,k)==0)
%                 plot([(t+c*141-11) (t+1+c*141-11)],[k k],'LineWidth',2)
%                 hold on
%             end
%         end
%     end
% end
% 
% axis([0 b*p-(11*p) 0.5 8.5])
% hold off
% grid on


%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% % porównanie nóg L i R
% % przesuniêcie 70 klatek !!!!
% figure;
% axes = axes('YTickLabel',{'nogi 1','nogi 2','nogi 3','nogi 4'},'YTick',[1.05 2.05 3.05 4.05],'YGrid','off',...
%             'XTickLabel',{'0','20','40','60','80','100'},'XTick',[11 39 67 95 123 152],'XGrid','on');
%         
% box(axes,'on');
% hold(axes,'all');
% 
% % nogi R
% for k=1:4
%     for t=a:b-1
%         if(g(t,k)==0 && g(t+1,k)==0)
%             plot([t t+1],[k k],'LineWidth',2)
%             hold on
%         end
%     end
% end
% 
% % nogi L
% s = 3.9;
% for k=5:8
%     for t=a:b-1
%         if(g(t,k)==0 && g(t+1,k)==0)
%             if(t+70>151)
%                 tn=t-141;
%             else
%                 tn=t;
%             end
%             plot([tn+70 tn+1+70],[k-s k-s],'r','LineWidth',2)
%             hold on
%         end
%     end
% end
% 
% axis([a b 0.5 4.5])
% xlabel('% cyklu')
% hold off


%% 
dlugosc_kroku_COM = COM(b,2)-COM(a,2); % mm
% dlugosc_kroku_R1 = R(b,2,1,1)-R(a,2,1,1) % mm
czas_kroku = (b-a)*0.008; % s
czestotliwosc = 1/czas_kroku; % 1/s
srednia_predkosc_COM = dlugosc_kroku_COM/czas_kroku/1000; % m/s

% wspólczynnik czasu gdy noga jest w powietrzu do czasu gdy stoi
% kolejnoœæ R1 R2 R3 R4 L1 L2 L3 L4

df = zeros(8,1);
for k=1:8
    q = 0;
    for t=a:b-1
        
        if(g(t,k)==1)
        q = q+1;
        end

    end
    
    df(k) = q/(b-a);
end


%% 
% k¹ty pomiêdzy kolejnymi czêœciami nogi zgodnie z rysunkiem 'oznaczenia k¹tów'
% teta miêdzy p³aszczyzn¹ p³aszczyzn¹ przecinaj¹c¹ paj¹ka na pó³ a p³aszczyzn¹ przechodz¹c¹ prze stawy 1 i 4
% kolejnoœæ R1 R2 R3 R4 L1 L2 L3 L4

alpha = zeros(185,8);
beta = zeros(185,8);
gama = zeros(185,8);
teta = zeros(185,8);
Rn = zeros(185,3,4,4);
Ln = zeros(185,3,4,4);


for t=1:185
    
    % trick na obliczenie k¹ta teta - obracam wspó³¿êdne x,y stawu 3 i 4 
    % tak aby p³aczczyzna przecinaj¹ca paj¹ka na pó³ by³a równoleg³a do osi X 
    p1=(R(t,:,1,4)+L(t,:,1,4))/2;
    p2=(R(t,:,4,4)+L(t,:,4,4))/2;
    fi = pi-atan2((p1(1)-p2(1)), (p2(2)-p1(2)));
    ROT = [cos(fi), -sin(fi); sin(fi), cos(fi)];
        
    for k=1:4

        alpha(t,k) = angle(R(t,:,k,2),R(t,:,k,3),R(t,:,k,4));
        alpha(t,k+4) = angle(L(t,:,k,2),L(t,:,k,3),L(t,:,k,4));
        
        beta(t,k) = angle(R(t,:,k,1),R(t,:,k,2),R(t,:,k,3));
        beta(t,k+4) = angle(L(t,:,k,1),L(t,:,k,2),L(t,:,k,3));
        
        gama(t,k) = angle(R(t,:,k,3),R(t,:,k,4),[R(t,1:2,k,3) R(t,:,k,4)]);
        gama(t,k+4) = angle(L(t,:,k,3),L(t,:,k,4),[L(t,1:2,k,3) L(t,:,k,4)]);
        
        
        Rn(t,1:2,k,4) = (ROT * [R(t,1,k,4); R(t,2,k,4)])';
        Rn(t,1:2,k,1) = (ROT * [R(t,1,k,1); R(t,2,k,1)])';
        teta(t,k) = angle([Rn(t,1,k,4) (Rn(t,2,k,4)+10) R(t,3,k,4)], [Rn(t,1:2,k,4) R(t,3,k,4)], [Rn(t,1:2,k,1) R(t,3,k,4)]);
        
        Ln(t,1:2,k,4) = (ROT * [L(t,1,k,4); L(t,2,k,4)])';
        Ln(t,1:2,k,1) = (ROT * [L(t,1,k,1); L(t,2,k,1)])';
        teta(t,k+4) = angle([Ln(t,1,k,4) (Ln(t,2,k,4)+10) L(t,3,k,4)], [Ln(t,1:2,k,4) L(t,3,k,4)], [Ln(t,1:2,k,1) L(t,3,k,4)]);
       
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%% wykresy
u = [98 52 22 103 28 135 90 30]; % momenty (w klatkach) uniesiania nogi - odczytane z wykresu
p = [152 93 48 125 82 25 123 65]; % momenty (w klatkach) postawienia nogi - odczytane z wykresu


% %%%%%%% nogi R
% figure;
% for i=1:4
%     
%     if (i==1)
%         sub = subplot(2,2,1,'XTickLabel',{'0','u','p'},'XTick',[11 u(i) p(i)],'XGrid','on');
%     else
%         sub = subplot(2,2,i,'XTickLabel',{'0','u','p','100'},'XTick',[11 u(i) p(i) 152],'XGrid','on');
%     end
%     
%     xlim(sub,[a b]);
%     ylim(sub,[0 180]);
%     hold(sub,'all');
%     
%     plot(a:b, alpha(a:b,i),'Color','b');
%     plot(a:b, beta(a:b,i),'Color','b','LineStyle','--');
%     plot(a:b, gama(a:b,i),'Color','b','LineStyle',':')
%     plot(a:b, teta(a:b,i),'Color','b','LineStyle','-.')
%     
%     title(['R', num2str(i)])
%     ylabel('k¹t [deg]')
%     xlabel('% cyklu')
%     
% end
% 
% 
% %%%%%%%% nogi L
% figure;
% for i=5:8
% 
%     if (i==6)
%         sub = subplot(2,2,2,'XTickLabel',{'0','p','u','100'},'XTick',[11 p(i) u(i) 152],'XGrid','on');
%     else
%         sub = subplot(2,2,i-4,'XTickLabel',{'0','u','p','100'},'XTick',[11 u(i) p(i) 152],'XGrid','on');
%     end
%     
%     xlim(sub,[a b]);
%     ylim(sub,[0 180]);
%     hold(sub,'all');
%     
%     plot(a:b, alpha(a:b,i),'Color','b');
%     plot(a:b, beta(a:b,i),'Color','b','LineStyle','--');
%     plot(a:b, gama(a:b,i),'Color','b','LineStyle',':')
%     plot(a:b, teta(a:b,i),'Color','b','LineStyle','-.')
%     
%     title(['L', num2str(i-4)])
%     ylabel('k¹t [deg]')
%     xlabel('% cyklu')
%     
% end


%% 
% % porównanie k¹tów dla nóg R i L
% figure;
% for i=1:2
%     
%     sub = subplot(2,2,i,'XTickLabel',{'0','100'},'XTick',[11 152],'XGrid','on');
%     
%     xlim(sub,[a b]);
%     ylim(sub,[0 180]);
%     hold(sub,'all');
%     
%     plot(a:b, alpha(a:b,i),'Color','b');
%     plot(a:b, beta(a:b,i),'Color','b','LineStyle','--');
%     
%     p = 70;
%     plot(a:a+p, alpha(b-p:b,i+4),'Color','r');
%     plot(a+p:b, alpha(a:b-p,i+4),'Color','r');
%     plot(a:a+p, beta(b-p:b,i+4),'Color','r','LineStyle','--');
%     plot(a+p:b, beta(a:b-p,i+4),'Color','r','LineStyle','--');
%     
%     title(['nogi ', num2str(i)])
%     ylabel('k¹t [deg]')
%     xlabel('% cyklu')
%     
%     
%     
%     
%     sub = subplot(2,2,i+2,'XTickLabel',{'0','100'},'XTick',[11 152],'XGrid','on');
%     
%     xlim(sub,[a b]);
%     ylim(sub,[0 180]);
%     hold(sub,'all');
%     
%     plot(a:b, gama(a:b,i),'Color','b','LineStyle',':')
%     plot(a:b, teta(a:b,i),'Color','b','LineStyle','-.')
%     
%     p = 70;
%     plot(a:a+p, gama(b-p:b,i+4),'Color','r','LineStyle',':')
%     plot(a+p:b, gama(a:b-p,i+4),'Color','r','LineStyle',':');
%     plot(a:a+p, teta(b-p:b,i+4),'Color','r','LineStyle','-.')
%     plot(a+p:b, teta(a:b-p,i+4),'Color','r','LineStyle','-.');
%     
%     ylabel('k¹t [deg]')
%     xlabel('% cyklu')
%     
% end
% 
% 
% figure;
% for i=3:4
%     
%     sub = subplot(2,2,i-2,'XTickLabel',{'0','100'},'XTick',[11 152],'XGrid','on');
%     
%     xlim(sub,[a b]);
%     ylim(sub,[0 180]);
%     hold(sub,'all');
%     
%     plot(a:b, alpha(a:b,i),'Color','b');
%     plot(a:b, beta(a:b,i),'Color','b','LineStyle','--');
%     
%     p = 70;
%     plot(a:a+p, alpha(b-p:b,i+4),'Color','r');
%     plot(a+p:b, alpha(a:b-p,i+4),'Color','r');
%     plot(a:a+p, beta(b-p:b,i+4),'Color','r','LineStyle','--');
%     plot(a+p:b, beta(a:b-p,i+4),'Color','r','LineStyle','--');
%     
%     title(['nogi ', num2str(i)])
%     ylabel('k¹t [deg]')
%     xlabel('% cyklu')
%     
%     
%     
%     
%     sub = subplot(2,2,i,'XTickLabel',{'0','100'},'XTick',[11 152],'XGrid','on');
%     
%     xlim(sub,[a b]);
%     ylim(sub,[0 180]);
%     hold(sub,'all');
%     
%     plot(a:b, gama(a:b,i),'Color','b','LineStyle',':')
%     plot(a:b, teta(a:b,i),'Color','b','LineStyle','-.')
%     
%     p = 70;
%     plot(a:a+p, gama(b-p:b,i+4),'Color','r','LineStyle',':')
%     plot(a+p:b, gama(a:b-p,i+4),'Color','r','LineStyle',':');
%     plot(a:a+p, teta(b-p:b,i+4),'Color','r','LineStyle','-.')
%     plot(a+p:b, teta(a:b-p,i+4),'Color','r','LineStyle','-.');
%     
%     ylabel('k¹t [deg]')
%     xlabel('% cyklu')
%     
% end

%% 
% porównanie nóg 1 i 2 oraz 3 i 4 dla R i L

% figure;
% 
% % R1 i R2
% sub = subplot(2,2,1,'XTickLabel',{'0','100'},'XTick',[11 152],'XGrid','on');
% 
% xlim(sub,[a b]);
% ylim(sub,[0 180]);
% hold(sub,'all');
% 
% plot(a:b, alpha(a:b,1),'Color','b');
% plot(a:b, beta(a:b,1),'Color','b','LineStyle','--');
% 
% p = 46;
% plot(a:a+p, alpha(b-p:b,2),'Color','r');
% plot(a+p:b, alpha(a:b-p,2),'Color','r');
% plot(a:a+p, beta(b-p:b,2),'Color','r','LineStyle','--');
% plot(a+p:b, beta(a:b-p,2),'Color','r','LineStyle','--');
% 
% title('R1 i R2')
% ylabel('k¹t [deg]')
% xlabel('% cyklu')
% 
% sub = subplot(2,2,3,'XTickLabel',{'0','100'},'XTick',[11 152],'XGrid','on');
% 
% xlim(sub,[a b]);
% ylim(sub,[0 180]);
% hold(sub,'all');
% 
% plot(a:b, gama(a:b,1),'Color','b','LineStyle',':')
% plot(a:b, teta(a:b,1),'Color','b','LineStyle','-.')
% 
% p = 45;
% plot(a:a+p, gama(b-p:b,2),'Color','r','LineStyle',':')
% plot(a+p:b, gama(a:b-p,2),'Color','r','LineStyle',':');
% plot(a:a+p, teta(b-p:b,2),'Color','r','LineStyle','-.')
% plot(a+p:b, teta(a:b-p,2),'Color','r','LineStyle','-.');
% 
% ylabel('k¹t [deg]')
% xlabel('% cyklu')
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % R3 i R4
% sub = subplot(2,2,2,'XTickLabel',{'0','100'},'XTick',[11 152],'XGrid','on');
% 
% xlim(sub,[a b]);
% ylim(sub,[0 180]);
% hold(sub,'all');
% 
% plot(a:b, alpha(a:b,3),'Color','b');
% plot(a:b, beta(a:b,3),'Color','b','LineStyle','--');
% 
% p = 71;
% plot(a:a+p, alpha(b-p:b,4),'Color','r');
% plot(a+p:b, alpha(a:b-p,4),'Color','r');
% plot(a:a+p, beta(b-p:b,4),'Color','r','LineStyle','--');
% plot(a+p:b, beta(a:b-p,4),'Color','r','LineStyle','--');
% 
% title('R3 i R4')
% ylabel('k¹t [deg]')
% xlabel('% cyklu')
% 
% sub = subplot(2,2,4,'XTickLabel',{'0','100'},'XTick',[11 152],'XGrid','on');
% 
% xlim(sub,[a b]);
% ylim(sub,[0 180]);
% hold(sub,'all');
% 
% plot(a:b, gama(a:b,3),'Color','b','LineStyle',':')
% plot(a:b, teta(a:b,3),'Color','b','LineStyle','-.')
% 
% p = 71;
% plot(a:a+p, gama(b-p:b,4),'Color','r','LineStyle',':')
% plot(a+p:b, gama(a:b-p,4),'Color','r','LineStyle',':');
% plot(a:a+p, teta(b-p:b,4),'Color','r','LineStyle','-.')
% plot(a+p:b, teta(a:b-p,4),'Color','r','LineStyle','-.');
% 
% ylabel('k¹t [deg]')
% xlabel('% cyklu')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% figure;
% 
% % L1 i L2
% sub = subplot(2,2,1,'XTickLabel',{'0','100'},'XTick',[11 152],'XGrid','on');
% 
% xlim(sub,[a b]);
% ylim(sub,[0 180]);
% hold(sub,'all');
% 
% plot(a:b, alpha(a:b,5),'Color','b');
% plot(a:b, beta(a:b,5),'Color','b','LineStyle','--');
% 
% p = 45;
% plot(a:a+p, alpha(b-p:b,6),'Color','r');
% plot(a+p:b, alpha(a:b-p,6),'Color','r');
% plot(a:a+p, beta(b-p:b,6),'Color','r','LineStyle','--');
% plot(a+p:b, beta(a:b-p,6),'Color','r','LineStyle','--');
% 
% title('L1 i L2')
% ylabel('k¹t [deg]')
% xlabel('% cyklu')
% 
% sub = subplot(2,2,3,'XTickLabel',{'0','100'},'XTick',[11 152],'XGrid','on');
% 
% xlim(sub,[a b]);
% ylim(sub,[0 180]);
% hold(sub,'all');
% 
% plot(a:b, gama(a:b,5),'Color','b','LineStyle',':')
% plot(a:b, teta(a:b,5),'Color','b','LineStyle','-.')
% 
% p = 45;
% plot(a:a+p, gama(b-p:b,6),'Color','r','LineStyle',':')
% plot(a+p:b, gama(a:b-p,6),'Color','r','LineStyle',':');
% plot(a:a+p, teta(b-p:b,6),'Color','r','LineStyle','-.')
% plot(a+p:b, teta(a:b-p,6),'Color','r','LineStyle','-.');
% 
% ylabel('k¹t [deg]')
% xlabel('% cyklu')
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % L3 i L4
% sub = subplot(2,2,2,'XTickLabel',{'0','100'},'XTick',[11 152],'XGrid','on');
% 
% xlim(sub,[a b]);
% ylim(sub,[0 180]);
% hold(sub,'all');
% 
% plot(a:b, alpha(a:b,7),'Color','b');
% plot(a:b, beta(a:b,7),'Color','b','LineStyle','--');
% 
% p = 60;
% plot(a:a+p, alpha(b-p:b,8),'Color','r');
% plot(a+p:b, alpha(a:b-p,8),'Color','r');
% plot(a:a+p, beta(b-p:b,8),'Color','r','LineStyle','--');
% plot(a+p:b, beta(a:b-p,8),'Color','r','LineStyle','--');
% 
% title('L3 i L4')
% ylabel('k¹t [deg]')
% xlabel('% cyklu')
% 
% sub = subplot(2,2,4,'XTickLabel',{'0','100'},'XTick',[11 152],'XGrid','on');
% 
% xlim(sub,[a b]);
% ylim(sub,[0 180]);
% hold(sub,'all');
% 
% plot(a:b, gama(a:b,7),'Color','b','LineStyle',':')
% plot(a:b, teta(a:b,7),'Color','b','LineStyle','-.')
% 
% p = 60;
% plot(a:a+p, gama(b-p:b,8),'Color','r','LineStyle',':')
% plot(a+p:b, gama(a:b-p,8),'Color','r','LineStyle',':');
% plot(a:a+p, teta(b-p:b,8),'Color','r','LineStyle','-.')
% plot(a+p:b, teta(a:b-p,8),'Color','r','LineStyle','-.');
% 
% ylabel('k¹t [deg]')
% xlabel('% cyklu')

