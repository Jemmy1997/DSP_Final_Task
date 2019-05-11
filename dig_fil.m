function varargout = dig_fil(varargin)
% DIG_FIL MATLAB code for dig_fil.fig
%      DIG_FIL, by itself, creates a new DIG_FIL or raises the existing
%      singleton*.
%
%      H = DIG_FIL returns the handle to a new DIG_FIL or the handle to
%      the existing singleton*.
%
%      DIG_FIL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DIG_FIL.M with the given input arguments.
%
%      DIG_FIL('Property','Value',...) creates a new DIG_FIL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before dig_fil_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to dig_fil_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help dig_fil

% Last Modified by GUIDE v2.5 11-May-2019 23:05:53

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @dig_fil_OpeningFcn, ...
                   'gui_OutputFcn',  @dig_fil_OutputFcn, ...
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


% --- Executes just before dig_fil is made visible.
function dig_fil_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to dig_fil (see VARARGIN)
DrawUnitCircle(hObject, eventdata, handles);

handles.i=1;
handles.min=0;
handles.scale=3;
handles.max=handles.scale;
set(handles.Start_Filtering,'Enable','off');
set(handles.slider1, 'Enable', 'off');
set(handles.slider1, 'Enable', 'off');
set(handles.slider2, 'Enable', 'off');

guidata(hObject, handles);
 
t=[0:0.1:20];
A=0.5;
fa=1;
y=A*cos(fa*t);
%calculate signal fft
handles.y_fft=fft(y);

handles.p=[];   % Array holds poles in complex formula
handles.z=[];   % Array holds zeros in complex formula
% Choose default command line output for unit_circle
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);



% UIWAIT makes dig_fil wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = dig_fil_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes1
function DrawUnitCircle(hObject, eventdata, handles)
axes(handles.axes1);
handles.ang = 0:0.01:2*pi;
x = cos(handles.ang);
y = sin(handles.ang);
grid on  
axis('equal');
xlabel('Real');
ylabel('Imaginary');
plot(handles.axes1,x,y,'k','LineWidth',1);
hold on
plot( [0 0], [1.5 -1.5], '-')
plot( [1.5 -1.5], [0 0], '-')
xlim([-1.5 1.5])
ylim([-1.5 1.5])
hold off;

% Update handles structure
guidata(hObject, handles);

function freq_plot(hObject, eventdata, handles)
%clear the unit circuit axes
cla(handles.axes1,'reset');
handles.fmax = 50
axes(handles.axes1)
DrawUnitCircle(hObject, eventdata, handles);
hold on
%plot poles and zeros markers
plot_p=plot(real(handles.p),imag(handles.p),'X');
plot_z=plot(real(handles.z),imag(handles.z),'O');
set(plot_p,'markersize',8,'linewidth',1.5,'Color', [255 100 100]/255);
set(plot_z,'markersize',8,'linewidth',1.5,'Color', [105 100 200]/255);
hold off;
%Get the transfer function coeffecients
[num,den]= zp2tf(handles.z',handles.p,1);
setappdata(0,'num_g',num)
setappdata(0,'den_g',den)

%Get the frequency response 
[handles.h,handles.w] = freqz(num,den,length(handles.y_fft));

%plot the frequency response mag 

if (isempty(handles.p) && isempty(handles.z))
    cla(handles.axes2,'reset')
else
    axes(handles.axes2)
   %  plot(handles.w/pi,(abs(handles.h)))
    plot((handles.w/pi)*handles.fmax,20*log(abs(handles.h)),'r-');
    xlabel('Normalized Frequency (\times\pi rad/sample)')
    ylabel('Magnitude (dB)')
    grid on;
end
 
% Update handles structure
guidata(hObject, handles);



% --- Executes on button press in Add_Pole_Conj.
function Add_Pole_Conj_Callback(hObject, eventdata, handles)
% hObject    handle to Add_Pole_Conj (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[x,y]=ginput(1);
%push a point and its conjugate to the poles array 
handles.p(length(handles.p)+1)=x+1j*y;
handles.p(length(handles.p)+1)=x+1j*(-y);
% Update handles structure
guidata(hObject, handles);
%plot the freq response and its effect in the original signal
freq_plot(hObject, eventdata, handles);
% Update handles structure
guidata(hObject, handles);

%Add Pole 
% --- Executes on button press in Add_Pole.
function Add_Pole_Callback(hObject, eventdata, handles)

[x,y]=ginput(1);
%push a point and its conjugate to the poles array 
handles.p(length(handles.p)+1)=x+1j*y;
% Update handles structure
guidata(hObject, handles);
%plot the freq response and its effect in the original signal
freq_plot(hObject, eventdata, handles);
% Update handles structure
guidata(hObject, handles);

% hObject    handle to Add_Pole (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in Delete_Pole_Conj.
function Delete_Pole_Conj_Callback(hObject, eventdata, handles)
% hObject    handle to Delete_Pole_Conj (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[x,y]=ginput(1);
%search for the selected pole from the poles array and remove it
temp=find(real(handles.p <(x+0.1)) & real(handles.p>(x-0.1)) );
handles.p(find(real(handles.p <(x+0.1)) & real(handles.p>(x-0.1)) ))=[];
% Update handles structure
guidata(hObject, handles);
%plot the freq response and its effect in the original signal
freq_plot(hObject, eventdata, handles);
% Update handles structure
guidata(hObject, handles);


%Add Conjugate zeros
% --- Executes on button press in Add_Zero_Conj.
function Add_Zero_Conj_Callback(hObject, eventdata, handles)
% hObject    handle to Add_Zero_Conj (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[x,y]=ginput(1);
%push a point to the zeros array 
handles.z(length(handles.z)+1)=x+1j*y;
handles.z(length(handles.z)+1)=x+1j*(-y);
% Update handles structure
guidata(hObject, handles);
%plot the freq response and its effect in the original signal
freq_plot(hObject, eventdata, handles);
% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in Delete_Zero_Conj.
function Delete_Zero_Conj_Callback(hObject, eventdata, handles)
% hObject    handle to Delete_Zero_Conj (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[x,y]=ginput(1);
%search for the selected zero from the zero array and remove it
temp=find((real(handles.z <(x+0.1)) & real(handles.z>(x-0.1))));
handles.z(find((real(handles.z <(x+0.1)) & real(handles.z>(x-0.1)))))=[];
% Update handles structure
guidata(hObject, handles);
%plot the freq response and its effect in the original signal
freq_plot(hObject, eventdata, handles);
% Update handles structure
guidata(hObject, handles);

%%% Add zeros 
% --- Executes on button press in Add_Zero.
function Add_Zero_Callback(hObject, eventdata, handles)
% hObject    handle to Add_Zero (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[x,y]=ginput(1);
%push a point to the zeros array 
handles.z(length(handles.z)+1)=x+1j*y;
% Update handles structure
guidata(hObject, handles);
%plot the freq response and its effect in the original signal
freq_plot(hObject, eventdata, handles);
% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in Delete_Pole.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to Delete_Pole (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in Clear_All.
function Clear_All_Callback(hObject, eventdata, handles)
% hObject    handle to Clear_All (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.p=[];
handles.z=[];

%clear all plots
cla(handles.axes1,'reset')
cla(handles.axes2,'reset')
cla(handles.axes3,'reset')
cla(handles.axes4,'reset')
cla(handles.axes5,'reset')
DrawUnitCircle(hObject, eventdata, handles)
% Update handles structure
guidata(hObject, handles);


% % % --- Executes on button press in MY_Gain.
 



% --- Executes on button press in Record.
% function Record_Callback(hObject, eventdata, handles)
% % hObject    handle to Record (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% flag = get(handles.Record,'Value');
%  while (flag)
% recorder = audiorecorder;
% get(recorder);
% recordblocking(recorder, 0.2);
% set(handles.Record,'string','Stop...');
% pause(.07);
% myRecording = getaudiodata(recorder);
% 
% %filter voice
% b = getappdata(0,'num_g');
% a = getappdata(0,'den_g');
% myRecording_filtered = filter(b,a,myRecording);
% %plot voice
% axes(handles.axes4);
% y=0;
% y=[y,myRecording.'];
% plot (y)
% %plot voice after filter 
% axes(handles.axes5)
% plot(myRecording_filtered);
% 
% flag = get(handles.Record,'value');
% if flag==0
%     set(handles.Record,'string','Start...');
% end
% end


% --- Executes on button press in Browse_Button.
function Browse_Button_Callback(hObject, eventdata, handles)
% hObject    handle to Browse_Button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 [filename] = uigetfile({'*.mat'},'file selector');
 signal = load(filename);
 handles.i = 1
 freq = 200
 handles.ECGsignal = (signal.val)/200;
 handles.time = (0:length(handles.ECGsignal)-1)/freq;
 set(handles.Start_Filtering,'Enable','on')
 set(handles.hand_gain, 'Enable', 'on');
 handles.Filtered = abs(handles.h').*handles.ECGsignal;
 guidata(hObject, handles);



function Sampling_Box_Callback(hObject, eventdata, handles)
% hObject    handle to Sampling_Box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Sampling_Box as text
%        str2double(get(hObject,'String')) returns contents of Sampling_Box as a double


% --- Executes during object creation, after setting all properties.
function Sampling_Box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Sampling_Box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
ax = axis;
scale = ax(2)-ax(1);
val = get(handles.slider1, 'value');
step = get(handles.slider1, 'SliderStep');
x1 = val+step(1);
x2 = x1+scale;
axis([x1 x2 ax(3) ax(4)]);
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in Start_Filtering.
function Start_Filtering_Callback(hObject, ~, handles)
% hObject    handle to Start_Filtering (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Start_Filtering


set(handles.slider1, 'Enable', 'on');
set(handles.slider2, 'Enable', 'on');
x=linspace(0,100000,10000000);
axes(handles.axes4)
norm=animatedline;
axes(handles.axes5)
filtered = animatedline;
ax=gca;
ax.XLimMode='manual';
ax.XLim=[handles.min handles.max];

while (get(hObject,'Value'))&&( handles.i<length(handles.time)) 
    
    addpoints(norm,handles.time(handles.i),handles.ECGsignal(handles.i));
    addpoints(filtered,handles.time(handles.i),handles.filtered_signal(handles.i));
    
    if handles.time(handles.i)>handles.scale
        set(handles.slider1, 'Enable', 'inactive');
        handles.min=handles.time(handles.i)-handles.scale;
        handles.max=handles.time(handles.i);
        ax.XLim=[handles.min handles.max];
        set(handles.slider1, 'Max', handles.max-handles.scale);
        set(handles.slider1, 'value', handles.min);
        step = [.1/handles.max 1/handles.max];
        set(handles.slider1, 'SliderStep', step);
        if get(hObject,'Value')==0
           set(handles.Start_Filtering,'string','pause','enable','on');
        else
             set(handles.Start_Filtering,'string','start','enable','on');
        end
        
    end
    drawnow;
    handles.i= handles.i+1;
    guidata(hObject, handles);  %Get the newest GUI data
end
if get(hObject,'Value')==0
    set(handles.Start_Filtering,'string','pause','enable','on');
else
    set(handles.Start_Filtering,'string','start','enable','on');
end
set(handles.slider1, 'Enable', 'on');


guidata(hObject, handles);


% --- Executes on button press in hand_gain.
function hand_gain_Callback(hObject, eventdata, handles)
% hObject    handle to hand_gain (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.fsample = str2double(get(handles.Sampling_Box,'string'))
 handles.fmax = floor(handles.fsample/2);
 f = handles.fmax+1;
 handles.ang = 0:0.01:2*pi;
 g_z = ones(1,f);
 g_p=ones(1,f);
 g = ones(1,f);
 x=0:1:handles.fmax;
 a=0;
 b=pi;
 handles.ang= (x-min(x))*(b-a)/(max(x)-min(x)) + a ;
 for i = 1:length(handles.z)
     for j=1:f
         xp_uni_cir(j) = 1*cos(handles.ang(j));
         yp_uni_cir(j) = 1*sin(handles.ang(j));
         len0(j) = sqrt((xp_uni_cir(j)-real(handles.z(i)))^2 + (yp_uni_cir(j)-imag(handles.z(i)))^2 )
         l(i,j) = len0(j);
     end
  g_z(1,:) =g_z(1,:).*l(i,:);
 end
 for i = 1:length(handles.p)
     for j=1:f
         xp2_uni_cir(j) = 1*cos(handles.ang(j));
         yp2_uni_cir(j) = 1*sin(handles.ang(j));
         des(j) = sqrt((xp2_uni_cir(j)-real(handles.p(i)))^2 + (yp2_uni_cir(j)-imag(handles.p(i)))^2 )
         d(i,j) = des(j);
     end
  g_p(1,:) =g_p(1,:).*d(i,:);
 end
 g(1,:)=g_z(1,:)./g_p(1,:);
 axes(handles.axes3)
 xlabel('Normalized Frequency (\time\pi rad/sample)');
 ylabel('Magnitude(db)');
 title('Magnitude response (Gain)');
 plot(20*log(g),'b')
 grid on

 
 guidata(hObject, handles);
