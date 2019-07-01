
function varargout = Audio(varargin)

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Audio_OpeningFcn, ...
                   'gui_OutputFcn',  @Audio_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end

% --- Executes just before Audio is made visible.
function Audio_OpeningFcn(hObject, eventdata, handles, varargin)
radiobutton4_Callback(hObject, eventdata, handles)

axes(handles.axes1)
matlabImage = imread('Univalle.png');
image(matlabImage)
axis off
axis image
global volumen;
volumen=0.5;
set(handles.slider2,'Value',0.5);

handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

function varargout = Audio_OutputFcn(hObject, eventdata, handles) 

varargout{1} = handles.output;

function pushbutton1_Callback(hObject, eventdata, handles)
global y;
global Fs;
global h;

[file,path]=uigetfile({'*.mp3'},'Seleccionar archivo');
[y,Fs]=audioread([path file]);

mensaje=sprintf(file);
handles.text4.String=mensaje;
h=y;



% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
global player;
global filename;
global y;
global h;
global Fs;
player=audioplayer(h,Fs);
grafica();

function grafica()
global filename;
global filename1;
global h;
global y;
global Fs;
global volumen;
global player;

filename='cancion.mp4';
audiowrite(filename,h,44100);

frameLength = 50;
filename1='original.mp4';
audiowrite(filename1,y,44100);
fileReader = dsp.AudioFileReader( ...
   filename, ...
   'SamplesPerFrame',frameLength);
fileReader1 = dsp.AudioFileReader( ...
   filename1, ...
   'SamplesPerFrame',frameLength);
   scope = dsp.TimeScope( ...                  %<--- new lines of code
    'SampleRate',Fs, ... %<---
    'TimeSpan',1, ...                      %<---
    'BufferLength',1.5e6, ...               %<---
    'YLimits',[-1,1],'Title','Señal de salida','Position',[1 375  300 300]);  


scope1 = dsp.TimeScope( ...                  %<--- new lines of code
    'SampleRate',Fs, ... %<---
    'TimeSpan',1, ...                      %<---
    'BufferLength',1.5e6, ...               %<---
    'YLimits',[-1,1],'Title','Señal de entrada','Position', [1 1 300 300]);  


   play(player);
while isplaying(player)
signal = step(fileReader);
signal2=step(fileReader1);
    z=signal*volumen;
    zz=signal2*volumen;
   step(scope,z)
   step(scope1,zz)
   
end
   release(scope)  
   release(scope1)

% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA
 




% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
global player;
stop(player);
clear functions;
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
global volumen;
global volu;

volu =get(hObject,'Value');
SoundVolume(volu);
volumen=volu;

function slider2_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
global y;
global h;

h=flipud(y);


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
global h;
global y;
global Fs;

h=(y(:,1)+y(:,2))/2;




% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)

global y;
global h;
global Fs;
mono=zeros(size(y));
if isvector(y)==0
r=(y(:,1)+y(:,2))/2;
y=r;
h=[r,mono];
else
h=[y,mono];
end




% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
global y;
global Fs1;

[file,path]=uigetfile({'*.mp3'},'Seleccionar archivo');
[y,Fs1]=audioread([path file]);

mensaje2=sprintf(file);
handles.text8.String=mensaje2;

% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)

global y2;
global Fs2;

[file,path]=uigetfile({'*.mp3'},'Seleccionar archivo');
[y2,Fs2]=audioread([path file]);

mensaje3=sprintf(file);
handles.text9.String=mensaje3;

% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
global y;
global y2;
global Fs1;
global Fs2;
global h;
global Fs;
global p1;
global q1;
global c1;
global c2;
global p2;
global q2;

Fs=44100;
dato=str2double(get(handles.edit10,'String'));
dato2=str2double(get(handles.edit11,'String'));

[p1 q1]=rat(dato/Fs1,0.001);
c1=resample(y,p1,q1);

[p1 q1]=rat(Fs/dato,0.001);
c3=resample(c1,p1,q1);
l1=length(c3)

[p2 q2]=rat(dato2/Fs2,0.001);
c2=resample(y2,p2,q2);

[p2 q2]=rat(Fs/dato2,0.001);
c4=resample(c2,p2,q2);
l2=length(c4)

if l2<l1
    z2=zeros(l1,2);
    z2(1:l2,1:2)=c4;
    h=c3+z2;
elseif l2>l1
    z1=zeros(l2,2);
    z1(1:l1,1:2)=c3;
    h=c4+z1;
end
function edit3_Callback(hObject, eventdata, handles)

function edit3_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function pushbutton10_Callback(hObject, eventdata, handles)

global y;
global h;
Fs=44100;
dato=get(handles.edit3,'String');
tiempo=str2double(dato);
graba=audiorecorder(Fs,8,2)
inicio=msgbox('Grabación iniciada','Grabando');
recordblocking (graba,tiempo);
fin=msgbox('Grabación finalizada');
y=getaudiodata(graba);
h=y;

function pushbutton11_Callback(~, eventdata, handles)

global h;
global Fs;
global player;
global filename;
player=audioplayer(h,Fs)
filename='RecordedSound.mp4';
grafica();

function filtros()
global a;
global b;
global y;
global Fs;
global h;
global player;

iir=dsp.IIRFilter('Structure','Direct form I','Numerator',b,'Denominator',a)

h=step(iir,y);
%a=[1 -0.5772 0.4218 -0.0563];
%b=[0.0985 0.2956 2.2956 0.0985];

%a=[1 -0.5772 0.4218 -0.0563];
%b=[0.2569 -0.7707 0.7707 -0.2569];

%a=[1 -1.9425 2.1192 -1.2167 0.4128];
%b=[ 0.0675 0 -0.1349 0 0.0675];


function edit4_Callback(hObject, eventdata, handles)

function edit4_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)



% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function pushbutton12_Callback(hObject, eventdata, handles)
global a;
global b;
valor=get(handles.edit4,'String');
valor1=get(handles.edit5,'String');
a=valor;
a=str2num(a);
b=valor1;
b=str2num(b);
filtros();

function radiobutton4_CreateFcn(hObject, eventdata, handles)

function radiobutton5_CreateFcn(hObject, eventdata, handles)

function radiobutton5_Callback(hObject, eventdata, handles)

set(handles.pushbutton1,'enable','off');
set(handles.pushbutton7,'enable','off');
set(handles.pushbutton8,'enable','off');
set(handles.pushbutton9,'enable','off');
set(handles.pushbutton10,'enable','on');
set(handles.pushbutton11,'enable','on');
set(handles.edit3,'enable','on');

function radiobutton4_Callback(hObject, eventdata, handles)

set(handles.pushbutton1,'enable','on');
set(handles.pushbutton7,'enable','on');
set(handles.pushbutton8,'enable','on');
set(handles.pushbutton9,'enable','on');
set(handles.pushbutton10,'enable','off');
set(handles.pushbutton11,'enable','off');
set(handles.edit3,'enable','off');

function pushbutton13_Callback(hObject, eventdata, handles)
global f;

f=[0.0021315395931926 0.00631494409001077 -3.93557515468197e-18 -0.033017674584738 -0.0399354370217647 0.077103611011276 0.288031717169276 0.398742599485494 0.288031717169276 0.077103611011276 -0.0399354370217647 -0.033017674584738 -3.93557515468197e-18 0.00631494409001077 0.0021315395931926];
RespuestaFrecuencia();

function pushbutton14_Callback(hObject, eventdata, handles)
global f;
f=[-0.00213630830828813 -0.00632907198578416 1.38053296063167e-17 0.0330915422640973 0.0400247812319218 -0.0772761084656623 -0.288676106418269 0.599452009385428 -0.288676106418269 -0.0772761084656623 0.0400247812319218 0.0330915422640973 1.38053296063167e-17 -0.00632907198578416 -0.00213630830828813];
RespuestaFrecuencia();


function pushbutton15_Callback(hObject, eventdata, handles)
global f;
f=[0.00157140799443589 0.0109372940914829 0.0238229277632931 -1.15685886521416e-17 -0.0898639796450302 -0.158878268764266 -0.0679845405739257 0.151638449130599 0.271792695998968 0.151638449130599 -0.0679845405739257 -0.158878268764266 -0.0898639796450302 -1.15685886521416e-17 0.0238229277632931 0.0109372940914829 0.00157140799443589]; 
RespuestaFrecuencia();

function RespuestaFrecuencia()
global y;
global h;
global Fs;
global f;
y1=(y(:,1)+y(:,2))/2;
N1=length(y1);
N2=length(f);
numero=(N1+N2-1);
fft_y1=fft(y1,N1+N2-1);
ffty1norm=fft(y1,N1+N2-1)/(N1+N2-1);
fft_f=fft(f,N1+N2-1);
y2=fft_y1.*fft_f.';
y2norm=(fft_y1.*fft_f.')/(N1+N2-1);
h=ifft(y2);
angle_fft=(angle(y2)*180)/pi;
%angle_fft=unwrap(angle(y2));
angle2=(angle(fft_y1.')*180)/pi;
%angle2=angle(fft_y1.');
hold on;
fn=0:Fs/numero:Fs-Fs/numero;
figure;
subplot(2,2,1);%2x1 posicion 2
plot(fn,abs(ffty1norm.'));% Señal x, Señal y
title('Magnitud vs Frecuencia Entrada');%titulo de la grafica 2
axis([0,Fs/2,-inf,inf]);
ylabel ( '|X(n)| ' );
xlabel ( 'Frecuencia(Hz)' );

subplot(2,2,3);%2x1 posicion 2
plot(fn,abs(y2norm));% Señal x, Señal y
title('Magnitud vs Frecuencia Salida');%titulo de la grafica 2
axis([0,Fs/2,-inf,inf]);
ylabel ( '| Y(n)| ' );
xlabel ( 'Frecuencia(Hz)' );

subplot(2,2,2);%2x1 posicion 1
plot(fn,angle2);% Señal X, Señal Y
title('Fase vs Frecuencia Entrada');%titulo de la grafica 2
axis([0,1,-inf,inf]);
ylabel ( 'Grados(°)' );
xlabel ( 'Frecuencia(Hz)' );

subplot(2,2,4);%2x1 posicion 2
plot(fn,angle_fft);% Señal x, Señal y
title('Fase vs Frecuencia Salida');%titulo de la grafica 2
axis([0,1,-inf,inf]);
ylabel ( 'Grados(°)' );
xlabel ( 'Frecuencia(w)' );

function edit10_Callback(hObject, eventdata, handles)

function edit10_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit11_Callback(hObject, eventdata, handles)

function edit11_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
