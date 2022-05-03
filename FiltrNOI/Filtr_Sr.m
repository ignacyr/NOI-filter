function varargout = Filtr_Sr(varargin)
% FILTR_SR MATLAB code for Filtr_Sr.fig
%      FILTR_SR, by itself, creates a new FILTR_SR or raises the existing
%      singleton*.
%
%      H = FILTR_SR returns the handle to a new FILTR_SR or the handle to
%      the existing singleton*.
%
%      FILTR_SR('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in FILTR_SR.M with the given input arguments.
%
%      FILTR_SR('Property','Value',...) creates a new FILTR_SR or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Filtr_Sr_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Filtr_Sr_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Filtr_Sr

% Last Modified by GUIDE v2.5 25-Jan-2021 01:02:51

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Filtr_Sr_OpeningFcn, ...
                   'gui_OutputFcn',  @Filtr_Sr_OutputFcn, ...
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
% End initialization code - DO NOT EDIT


% --- Executes just before Filtr_Sr is made visible.
function Filtr_Sr_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Filtr_Sr (see VARARGIN)

% Choose default command line output for Filtr_Sr
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Filtr_Sr wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Filtr_Sr_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
Filtr_rodzaj
close Filtr_Sr
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function Czest_g_Callback(hObject, eventdata, handles)
% hObject    handle to Czest_g (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Czest_g as text
%        str2double(get(hObject,'String')) returns contents of Czest_g as a double


% --- Executes during object creation, after setting all properties.
function Czest_g_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Czest_g (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in start.
function start_Callback(hObject, eventdata, handles)
fd=str2double (get(handles.Czest_d,'String'));
fg=str2double (get(handles.Czest_g,'String'));
fp=str2double (get(handles.Czest_p,'String')); 
pr=handles.prototyp
%sg=handles.sygnal;
rn=handles.rzad;
if strcmp (rn,'2')
    n=2;
elseif strcmp (rn,'3')
    n=3;
elseif strcmp (rn,'4')
    n=4;
elseif strcmp (rn,'5')
    n=5;
elseif strcmp (rn,'6')
    n=6;
elseif strcmp (rn,'7')
    n=7;
elseif strcmp (rn,'8')
    n=8;
elseif strcmp (rn,'9')
    n=9;
elseif strcmp (rn,'10')
    n=10;
end
%Wczytanie sygnału
u = handles.Data';

wd = 2*pi*fd;   % zamiana na pulsacjÄ™
wg = 2*pi*fg;   % zamiana na pulsacjÄ™
wp = 2*pi*fp;   % zamiana na pulsacjÄ™
Rp = 0.3; % pasmo przepustowe
Rs = 30; % pasmo zaporowe
N=size(u,1);
t=0:1/fp:(N-1)/fp;
Nw=500;
axes(handles.axes5)
plot(t(1:Nw),u(2091:Nw+2090),'k'); grid; 
xlabel('t');
ylabel('U');

if strcmp (pr,'Butterworth')
    [b,a] = butter(n,[wd wg],'bandpass','s'); % prototypowy filtr butterwortha (analogowy)
    [z,p,k] = butter(n,[wd wg],'bandpass','s');  % przeksztalcenie do postaci zpk
    
W = 0:.1: wp ; 
H=freqs(b,a,W);
f=W./(2*pi); % zamiana na czÄ™stotliwoĹ›Ä‡

[zd,pd,kd] = bilinear(z,p,k,fp); % transmitancja biliniowa
sos = zp2sos(zd,pd,kd); % przekształcenie do postaci second order sections
[Hz,Wz]=freqz(sos,512,fp);

% plot(f,abs(H)); grid on; % plot filtr analogowy

%[Mzd,Lzd]=bilinear(b,a,fp); % transformacja biliniowa
%[Hz,Wz]=freqz(Mzd,Lzd,512,fp);  % postac transmitancji byla niestabilna

axes(handles.axes1)
plot(f(1:5*wp),abs(H(1:5*wp)),'b'); hold on; % plot filtr analogowy (niebieski)
plot(Wz,abs(Hz),'r'); grid on; hold off; % plot filtr cyfrowy (czerwony)
xlabel('f');
ylabel('k');

elseif strcmp (pr,'Czebyszewa I typu')
    [b,a] = cheby1(n,Rp,[wd wg],'bandpass','s'); % prototypowy filtr czebyszewa 1 typu (analogowy)
    [z,p,k] = cheby1(n,Rp,[wd wg],'bandpass','s');  % przeksztalcenie do postaci zpk
    
W = 0:.1: wp ; 
H=freqs(b,a,W);
f=W./(2*pi); % zamiana na czÄ™stotliwoĹ›Ä‡

[zd,pd,kd] = bilinear(z,p,k,fp) % transmitancja biliniowa
sos = zp2sos(zd,pd,kd); % przekształcenie do postaci second order sections
[Hz,Wz]=freqz(sos,512,fp);

%[Mzd,Lzd]=bilinear(b,a,fp); % transformacja biliniowa
%[Hz,Wz]=freqz(Mzd,Lzd,512,fp); % postac transmitancji byla niestabilna

axes(handles.axes1)
plot(f(1:5*wp),abs(H(1:5*wp)),'b'); hold on; % plot filtr analogowy (niebieski)
plot(Wz,abs(Hz),'r'); grid on; hold off; % plot filtr cyfrowy (czerwony)
xlabel('f');
ylabel('k');

elseif strcmp (pr,'Czebyszewa II typu')
    [b,a] = cheby2(n,Rs,[wd wg],'bandpass','s'); % prototypowy filtr czebyszewa 2 typu (analogowy)
    [z,p,k] = cheby2(n,Rs,[wd wg],'bandpass','s');  % przeksztalcenie do postaci zpk
    
W = 0:.1: wp ; 
H=freqs(b,a,W);
f=W./(2*pi); % zamiana na czÄ™stotliwoĹ›Ä‡

[zd,pd,kd] = bilinear(z,p,k,fp) % transmitancja biliniowa
sos = zp2sos(zd,pd,kd); % przekształcenie do postaci second order sections
[Hz,Wz]=freqz(sos,512,fp);

%[Mzd,Lzd]=bilinear(b,a,fp); % transformacja biliniowa
%[Hz,Wz]=freqz(Mzd,Lzd,512,fp); % postac transmitancji byla niestabilna

axes(handles.axes1)
plot(f(1:5*wp),abs(H(1:5*wp)),'b'); hold on; % plot filtr analogowy (niebieski)
plot(Wz,abs(Hz),'r'); grid on; hold off; % plot filtr cyfrowy (czerwony)
xlabel('f');
ylabel('k');

elseif strcmp (pr,'Eliptyczny')
    [b,a] = ellip(n,Rp,Rs,[wd wg],'bandpass','s'); % prototypowy filtr eliptyczny (analogowy)
    [z,p,k] = ellip(n,Rp,Rs,[wd wg],'bandpass','s');  % przeksztalcenie do postaci zpk

W = 0:.1: wp ;
H = freqs(b,a,W);
f = W./(2*pi); % zamiana na czÄ™stotliwoĹ›Ä‡

[zd,pd,kd] = bilinear(z,p,k,fp) % transmitancja biliniowa
sos = zp2sos(zd,pd,kd); % przekształcenie do postaci second order sections
[Hz,Wz]=freqz(sos,512,fp);

%[Mzd,Lzd]=bilinear(b,a,fp); % transformacja biliniowa
%[Hz,Wz]=freqz(Mzd,Lzd,512,fp);  % postac transmitancji byla niestabilna

axes(handles.axes1)
plot(f(1:5*wp),abs(H(1:5*wp)),'b'); hold on; % plot filtr analogowy (niebieski)
plot(Wz,abs(Hz),'r'); grid on; hold off; % plot filtr cyfrowy (czerwony)
xlabel('f');
ylabel('k');
end

% fft przed filtracjÄ…
Le=length(t)-1;
u_fft = abs(fft(u))/(Le/2);
u_fft = u_fft(1:Le/2+1);
u_fft(2:end-1) = 2*u_fft(2:end-1);
ft = fp*(0:(Le/2))/Le;
axes(handles.axes4)
plot(ft,u_fft); grid on;
xlabel('f');
ylabel('U');

% filtracja Ĺ›rodkowo
%u_przefiltrowane = filter(Mzd,Lzd,u); % postac transmitancji niestabilna
u_przefiltrowane = sosfilt(sos,u);    % postac zpk ok
axes(handles.axes2)
plot(t(1:Nw),u_przefiltrowane(2091:Nw+2090),'k'); grid;
xlabel('t');
ylabel('U');

% fft po filtracji Ĺ›rodkowo
Le=length(t)-1;
u_fft_przefiltrowane = abs(fft(u_przefiltrowane))/(Le/2);
u_fft_przefiltrowane = u_fft_przefiltrowane(1:Le/2+1);
u_fft_przefiltrowane(2:end-1) = 2*u_fft_przefiltrowane(2:end-1);
ft = fp*(0:(Le/2))/Le;
axes(handles.axes3)
plot(ft,u_fft_przefiltrowane); grid on;
xlabel('f');
ylabel('U');


% hObject    handle to start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function Czest_d_Callback(hObject, eventdata, handles)
% hObject    handle to Czest_d (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Czest_d as text
%        str2double(get(hObject,'String')) returns contents of Czest_d as a double


% --- Executes during object creation, after setting all properties.
function Czest_d_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Czest_d (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Czest_p_Callback(hObject, eventdata, handles)
% hObject    handle to Czest_p (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Czest_p as text
%        str2double(get(hObject,'String')) returns contents of Czest_p as a double


% --- Executes during object creation, after setting all properties.
function Czest_p_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Czest_p (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in sygnal.
function sygnal_Callback(hObject, eventdata, handles)
v_sygnal=get(hObject,'Value');
s_sygnal=get(hObject,'String');
handles.sygnal=s_sygnal{v_sygnal}
guidata(hObject, handles);
% hObject    handle to sygnal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns sygnal contents as cell array
%        contents{get(hObject,'Value')} returns selected item from sygnal


% --- Executes during object creation, after setting all properties.
function sygnal_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sygnal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in prototyp.
function prototyp_Callback(hObject, eventdata, handles)
v_prototyp=get(hObject,'Value');
s_prototyp=get(hObject,'String');
handles.prototyp=s_prototyp{v_prototyp}
guidata(hObject, handles);
% hObject    handle to prototyp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns prototyp contents as cell array
%        contents{get(hObject,'Value')} returns selected item from prototyp


% --- Executes during object creation, after setting all properties.
function prototyp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to prototyp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in rzad.
function rzad_Callback(hObject, eventdata, handles)
v_rzad=get(hObject,'Value');
s_rzad=get(hObject,'String');
handles.rzad=s_rzad{v_rzad}
guidata(hObject, handles);
% hObject    handle to rzad (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns rzad contents as cell array
%        contents{get(hObject,'Value')} returns selected item from rzad


% --- Executes during object creation, after setting all properties.
function rzad_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rzad (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
[filename, pathname] = uigetfile({'*.txt'},'File Selector');
if ~ischar(filename)
    return;  % User aborted the file selection
end
file = fullfile(pathname, filename);
[fid, msg] = fopen(file, 'r');
if fid == -1
    error(msg);
end
Data = fscanf(fid, '%g\n', [1, inf]);  % Or how your file is formatted
fclose(fid);
handles.Data = Data;
guidata(hObject, handles);  % Store updated handles struct in the GUI
set(handles.text20, 'String', filename); %wyswietl nazwe sygnalu
