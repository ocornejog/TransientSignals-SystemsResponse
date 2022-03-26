function varargout = sistemas(varargin)
% SISTEMAS MATLAB code for sistemas.fig
%      SISTEMAS, by itself, creates a new SISTEMAS or raises the existing
%      singleton*.
%
%      H = SISTEMAS returns the handle to a new SISTEMAS or the handle to
%      the existing singleton*.
%
%      SISTEMAS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SISTEMAS.M with the given input arguments.
%
%      SISTEMAS('Property','Value',...) creates a new SISTEMAS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before sistemas_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to sistemas_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help sistemas

% Last Modified by GUIDE v2.5 23-Sep-2019 00:50:32

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @sistemas_OpeningFcn, ...
                   'gui_OutputFcn',  @sistemas_OutputFcn, ...
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


% --- Executes just before sistemas is made visible.
function sistemas_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to sistemas (see VARARGIN)

% Choose default command line output for sistemas
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes sistemas wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = sistemas_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function y3_Callback(hObject, eventdata, handles)
% hObject    handle to y3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y3 as text
%        str2double(get(hObject,'String')) returns contents of y3 as a double


% --- Executes during object creation, after setting all properties.
function y3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function xt_Callback(hObject, eventdata, handles)
% hObject    handle to xt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of xt as text
%        str2double(get(hObject,'String')) returns contents of xt as a double


% --- Executes during object creation, after setting all properties.
function xt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function cond_Callback(hObject, eventdata, handles)
% hObject    handle to cond (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of cond as text
%        str2double(get(hObject,'String')) returns contents of cond as a double


% --- Executes during object creation, after setting all properties.
function cond_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cond (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in GRAFICAR.
function GRAFICAR_Callback(hObject, eventdata, handles)
% hObject    handle to GRAFICAR (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global A XT ctrl
syms x t

clc
axes(handles.axes1)
set(handles.axes1, 'visible', 'on')
cla
A = str2num(get(handles.INTERVALO, 'String'));
XT = eval(get(handles.xt, 'String'));
x = linspace(min(A), max(A), 1000);
fx = 0;
for i=1:length(A)-1
    if mod(i, 2) == 1
    fx = fx+((x>=A(i))&(x<=A(i+1))).*subs(XT(i),x);
    else
    fx = fx+((x>A(i))&(x<A(i+1))).*subs(XT(i),x);
    end
end
plot(x, fx, 'Linewidth', 2);
grid on
xlabel('\bfTIEMPO');
ylabel('\bfAMPLITUD');
title('\bfx[t]','FontName','Cambria','FontSize',12)
T = max(x)-min(x);

% --- Executes on button press in CALCULAR.
function CALCULAR_Callback(hObject, eventdata, handles)
% hObject    handle to CALCULAR (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global A XT Y3 Y2 Y1 Y0 X3 X2 X1 X0 COND  
syms u y(t) x(t) a t a1
A = str2num(get(handles.INTERVALO, 'String'));
XT = eval(get(handles.xt, 'String'));
Y3 = str2num(get(handles.y3, 'String'));
Y2 = str2num(get(handles.y2, 'String'));
Y1 = str2num(get(handles.y1, 'String'));
Y0 = str2num(get(handles.y0, 'String'));
X3 = str2num(get(handles.x3, 'String'));
X2 = str2num(get(handles.x2, 'String'));
X1 = str2num(get(handles.x1, 'String'));
X0 = str2num(get(handles.x0, 'String'));

Dy = diff(y,t);
D2y = diff(y,t,2);
COND = eval(get(handles.cond, 'String'));
clc
axes(handles.axes2)
set(handles.axes2, 'visible', 'off')
cla
x(t)=dirac(t);
eqn=(Y3*diff(y,t,3)+Y2*diff(y,t,2)+Y1*diff(y,t,1)+Y0*y(t)==...
    X3*diff(x,t,3)+X2*diff(x,t,2)+X1*diff(x,t,1)+X0*x(t));
pretty(eqn)
a1=dsolve(eqn,COND)
pretty(a1)
x(t)=XT;
eqn=(Y3*diff(y,t,3)+Y2*diff(y,t,2)+Y1*diff(y,t,1)+Y0*y(t)==...
    X3*diff(x,t,3)+X2*diff(x,t,2)+X1*diff(x,t,1)+X0*x(t));
pretty(eqn)
a=dsolve(eqn,COND)
pretty(a)
P = strcat('$$', 'x(t) = ', char(latex(x(t))),'$$');
text('Interpreter','latex',...
	'String',P,...
	'Position',[0 .9],...
	'FontSize',14);
P = strcat('$$', 'h(t) = ', char(latex(a1)),'$$');
text('Interpreter','latex',...
	'String',P,...
	'Position',[0 .55],...
	'FontSize',14);
P = strcat('$$', 'y(t) = ', char(latex(a)),'$$');
text('Interpreter','latex',...
	'String',P,...
	'Position',[0 .2],...
	'FontSize',14);
opcion=get(handles.OPCION,'Value');
switch opcion
    case 1
        axes(handles.axes3)
        A = str2num(get(handles.INTERVALO, 'String'));
        u = linspace(min(A), max(A), 1000);
        fx = 0;
        for i=1:length(A)-1
          if mod(i, 2) == 1
          fx = fx+((u>=A(i))&(u<=A(i+1))).*subs(a1(i),u);
          else
          fx = fx+((u>A(i))&(u<A(i+1))).*subs(a1(i),u);
          end
        end
        plot(u, fx, 'Linewidth', 2);
        grid on
        xlabel('\bfTIEMPO');
        ylabel('\bfAMPLITUD');
        title('\bfh[t]','FontName','Cambria','FontSize',12)
    case 2
        axes(handles.axes3)
        A = str2num(get(handles.INTERVALO, 'String'));
        u = linspace(min(A), max(A), 1000);
        fx = 0;
        for i=1:length(A)-1
          if mod(i, 2) == 1
          fx = fx+((u>=A(i))&(u<=A(i+1))).*subs(a(i),u);
          else
          fx = fx+((u>A(i))&(u<A(i+1))).*subs(a(i),u);
          end
        end
        plot(u, fx, 'Linewidth', 2);
        grid on
        xlabel('\bfTIEMPO');
        ylabel('\bfAMPLITUD');
        title('\bfy[t]','FontName','Cambria','FontSize',12)
    case 3
        axes(handles.axes3)
        A = str2num(get(handles.INTERVALO, 'String'));
        u = linspace(min(A), max(A), 1000);
        fx = 0;
        for i=1:length(A)-1
          if mod(i, 2) == 1
          fx = fx+((u>=A(i))&(u<=A(i+1))).*subs(a(i),u);
          else
          fx = fx+((u>A(i))&(u<A(i+1))).*subs(a(i),u);
          end
        end
        plot(u, fx, 'Linewidth', 2,'Color','r','DisplayName','y(t)');
        grid on
        xlabel('\bfTIEMPO');
        ylabel('\bfAMPLITUD');
        title('\bfy[t]','FontName','Cambria','FontSize',12)
        hold on
        axes(handles.axes3)
        A = str2num(get(handles.INTERVALO, 'String'));
        u = linspace(min(A), max(A), 1000);
        fx = 0;
        for i=1:length(A)-1
          if mod(i, 2) == 1
          fx = fx+((u>=A(i))&(u<=A(i+1))).*subs(a1(i),u);
          else
          fx = fx+((u>A(i))&(u<A(i+1))).*subs(a1(i),u);
          end
        end
        plot(u, fx, 'Linewidth', 2,'Color',[1 0.55 0],'DisplayName','h(t)');
        grid on
        xlabel('\bfTIEMPO');
        ylabel('\bfAMPLITUD');
        title('\bfy[t]','FontName','Cambria','FontSize',12)
        hold off
        lgd = legend; 
end
%get(handles.listbox2,'value')
%P = strcat('$$', 'y(t) = ', char(latex(a)),'$$');
%set(handles.listbox2,'Interpreter','latex','String',P);
function y2_Callback(hObject, eventdata, handles)
% hObject    handle to y2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y2 as text
%        str2double(get(hObject,'String')) returns contents of y2 as a double


% --- Executes during object creation, after setting all properties.
function y2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function y1_Callback(hObject, eventdata, handles)
% hObject    handle to y1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y1 as text
%        str2double(get(hObject,'String')) returns contents of y1 as a double


% --- Executes during object creation, after setting all properties.
function y1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function y0_Callback(hObject, eventdata, handles)
% hObject    handle to y0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y0 as text
%        str2double(get(hObject,'String')) returns contents of y0 as a double


% --- Executes during object creation, after setting all properties.
function y0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function x3_Callback(hObject, eventdata, handles)
% hObject    handle to x3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x3 as text
%        str2double(get(hObject,'String')) returns contents of x3 as a double


% --- Executes during object creation, after setting all properties.
function x3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function x2_Callback(hObject, eventdata, handles)
% hObject    handle to x2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x2 as text
%        str2double(get(hObject,'String')) returns contents of x2 as a double


% --- Executes during object creation, after setting all properties.
function x2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function x1_Callback(hObject, eventdata, handles)
% hObject    handle to x1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x1 as text
%        str2double(get(hObject,'String')) returns contents of x1 as a double


% --- Executes during object creation, after setting all properties.
function x1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function x0_Callback(hObject, eventdata, handles)
% hObject    handle to x0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x0 as text
%        str2double(get(hObject,'String')) returns contents of x0 as a double


% --- Executes during object creation, after setting all properties.
function x0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function INTERVALO_Callback(hObject, eventdata, handles)
% hObject    handle to INTERVALO (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of INTERVALO as text
%        str2double(get(hObject,'String')) returns contents of INTERVALO as a double


% --- Executes during object creation, after setting all properties.
function INTERVALO_CreateFcn(hObject, eventdata, handles)
% hObject    handle to INTERVALO (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in listbox2.
function listbox2_Callback(hObject, eventdata, handles)
% hObject    handle to listbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox2


% --- Executes during object creation, after setting all properties.
function listbox2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in OPCION.
function OPCION_Callback(hObject, eventdata, handles)
% hObject    handle to OPCION (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns OPCION contents as cell array
%        contents{get(hObject,'Value')} returns selected item from OPCION


% --- Executes during object creation, after setting all properties.
function OPCION_CreateFcn(hObject, eventdata, handles)
% hObject    handle to OPCION (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
