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

% Last Modified by GUIDE v2.5 13-May-2019 15:46:07

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
global num den
DrawUnitCircle(hObject, eventdata, handles);
handles.i=1;
handles.min=0;
handles.scale=3;
handles.max=handles.scale;
set(handles.Start_Filtering,'Enable','off')
guidata(hObject, handles);
handles.p=[];
handles.z=[];
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
global time ECGsignal i h Filtered fmax num den
cla(handles.axes1,'reset');
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
[num,den]= zp2tf((handles.z)',handles.p,1);

setappdata(0,'num_g',num)
setappdata(0,'den_g',den)
 
% %Get the frequency response 
[h,w] = freqz(num,den,length(ECGsignal));

if (isempty(handles.p) && isempty(handles.z))
    cla(handles.axes2,'reset')
else
    axes(handles.axes2)
    plot((w/pi)*fmax,(abs(h)),'r-');
    xlabel('Frequency (rad)');
    ylabel('Gain');
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

 
% --- Executes on button press in Browse_Button.
function Browse_Button_Callback(hObject, eventdata, handles)
% hObject    handle to Browse_Button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global time ECGsignal i h  Filteredsignal time2 num den
[filename] = uigetfile({'*.mat'},'file selector');
 signal = load(filename);
 i = 1;
 freq = 200;
 ECGsignal = (signal.val)/200;
 time = (0:length(ECGsignal)-1)/freq;

 set(handles.Start_Filtering,'Enable','on')
 set(handles.hand_gain, 'Enable', 'on');


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


% --- Executes on button press in Start_Filtering.
function Start_Filtering_Callback(hObject, ~, handles)
% hObject    handle to Start_Filtering (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Start_Filtering

global time ECGsignal i Filteredsignal time2 num den

Filteredsignal = filter( num ,den,ECGsignal);

axes(handles.axes6)
plot(abs(fft(ECGsignal)))

axes(handles.axes7)
plot(abs(fft(Filteredsignal)))

axes(handles.axes4)
norm=animatedline;
axes(handles.axes5)
Filtered  = animatedline;
 

while (get(hObject,'Value'))&&( i<length(time)) 
    
   addpoints(norm,time(i),ECGsignal(i));
   addpoints(Filtered,time(i), Filteredsignal(i));
   
    
    if time(i)>handles.scale
       
        handles.min=time(i)-handles.scale;
        handles.max=time(i);
        step = [.1/handles.max 1/handles.max];
        if get(hObject,'Value')==0
           set(handles.Start_Filtering,'string','pause','enable','on');
        else
             set(handles.Start_Filtering,'string','start','enable','on');
        end
        
    end
    drawnow;
    i= i+1;
    guidata(hObject, handles);  %Get the newest GUI data
end
if get(hObject,'Value')==0
    set(handles.Start_Filtering,'string','pause','enable','on');
else
    set(handles.Start_Filtering,'string','start','enable','on');
end
guidata(hObject, handles);


% --- Executes on button press in hand_gain.
function hand_gain_Callback(hObject, eventdata, handles)
% hObject    handle to hand_gain (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global time ECGsignal i h Filtered fmax ang fsample f num den
fsample = str2double(get(handles.Sampling_Box,'string'))
 fmax = floor(fsample/2);
 f = fmax+1;
 ang = 0:0.01:pi;
 Zeros_Gain=ones(1,f);
Poles_Gain=ones(1,f);
Total_Gain=ones(1,f);
 x=0:1:fmax;
 a=0;
 b=pi;
 ang= (x-min(x))*(b-a)/(max(x)-min(x)) + a ;
 for i = 1:length(handles.z)
     for j=1:f
        Xpole_unit(j) = 1*cos(ang(j));
        Ypole_unit(j) = 1*sin(ang(j));
        length_zero(j)=sqrt((Xpole_unit(j)-real(handles.z(i)))^2+(Ypole_unit(j)-imag(handles.z(i)))^2 ) ;
        Total_length_zero(i,j)=length_zero(j);
     end
 Zeros_Gain(1,:)=Zeros_Gain(1,:).*Total_length_zero(i,:);
 end
 for i = 1:length(handles.p)
     for j=1:f
        Xpole_unit(j) = 1*cos(ang(j));
        Ypole_unit(j) = 1*sin(ang(j));
        length_Pole(j)=sqrt((Xpole_unit(j)-real(handles.p(i)))^2+(Ypole_unit(j)-imag(handles.p(i)))^2 ) ;
        Total_length_pole(i,j)=length_Pole(j);
     end
  Poles_Gain(1,:)=Poles_Gain(1,:).*Total_length_pole(i,:);
 end
 Total_Gain(1,:)=Zeros_Gain(1,:)./Poles_Gain(1,:);

 axes(handles.axes3)
 plot(abs(Total_Gain),'b')
 xlabel('Frequency (rad)');
 ylabel('Gain');
 
 grid on

 
 guidata(hObject, handles);


% --- Executes on button press in Move_Zero.
function Move_Zero_Callback(hObject, eventdata, handles)
% hObject    handle to Move_Zero (see GCBO)
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
[x,y]=ginput(1);
%push a point and its conjugate to the zeros array 
handles.z(length(handles.z)+1)=x+1j*y;
% Update handles structure
guidata(hObject, handles);
%plot the freq response and its effect in the original signal
freq_plot(hObject, eventdata, handles);

% Update handles structure
guidata(hObject, handles);




% --- Executes on button press in Move_Zero_Conj.
function Move_Zero_Conj_Callback(hObject, eventdata, handles)
% hObject    handle to Move_Zero_Conj (see GCBO)
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
[x,y]=ginput(1);
%push a point and its conjugate to the zeros array 
handles.z(length(handles.z)+1)=x+1j*y;
handles.z(length(handles.z)+1)=x+1j*(-y);
% Update handles structure
guidata(hObject, handles);
%plot the freq response and its effect in the original signal
freq_plot(hObject, eventdata, handles);

% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in Move_Pole.
function Move_Pole_Callback(hObject, eventdata, handles)
% hObject    handle to Move_Pole (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[x,y]=ginput(1);
%search for the selected zero from the zero array and remove it
temp=find((real(handles.p <(x+0.1)) & real(handles.p>(x-0.1))));
handles.p(find((real(handles.p <(x+0.1)) & real(handles.p>(x-0.1)))))=[];
% Update handles structure
guidata(hObject, handles);
%plot the freq response and its effect in the original signal
freq_plot(hObject, eventdata, handles);
% Update handles structure
guidata(hObject, handles);
[x,y]=ginput(1);
%push a point and its conjugate to the zeros array 
handles.p(length(handles.p)+1)=x+1j*y;
% Update handles structure
guidata(hObject, handles);
%plot the freq response and its effect in the original signal
freq_plot(hObject, eventdata, handles);

% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in Move_Pole_Conj.
function Move_Pole_Conj_Callback(hObject, eventdata, handles)
% hObject    handle to Move_Pole_Conj (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[x,y]=ginput(1);
%search for the selected zero from the zero array and remove it
temp=find((real(handles.p <(x+0.1)) & real(handles.p>(x-0.1))));
handles.p(find((real(handles.p <(x+0.1)) & real(handles.p>(x-0.1)))))=[];
% Update handles structure
guidata(hObject, handles);
%plot the freq response and its effect in the original signal
freq_plot(hObject, eventdata, handles);
% Update handles structure
guidata(hObject, handles);
[x,y]=ginput(1);
%push a point and its conjugate to the zeros array 
handles.p(length(handles.p)+1)=x+1j*y;
handles.p(length(handles.p)+1)=x+1j*(-y);
% Update handles structure
guidata(hObject, handles);
%plot the freq response and its effect in the original signal
freq_plot(hObject, eventdata, handles);

% Update handles structure
guidata(hObject, handles);

