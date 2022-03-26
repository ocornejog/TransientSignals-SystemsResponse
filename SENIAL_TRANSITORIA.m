function varargout = SENIAL_TRANSITORIA(varargin)
% SENIAL_TRANSITORIA MATLAB code for SENIAL_TRANSITORIA.fig
%      SENIAL_TRANSITORIA, by itself, creates a new SENIAL_TRANSITORIA or raises the existing
%      singleton*.
%
%      H = SENIAL_TRANSITORIA returns the handle to a new SENIAL_TRANSITORIA or the handle to
%      the existing singleton*.
%
%      SENIAL_TRANSITORIA('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SENIAL_TRANSITORIA.M with the given input arguments.
%
%      SENIAL_TRANSITORIA('Property','Value',...) creates a new SENIAL_TRANSITORIA or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before SENIAL_TRANSITORIA_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to SENIAL_TRANSITORIA_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help SENIAL_TRANSITORIA

% Last Modified by GUIDE v2.5 09-Sep-2016 22:42:08

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @SENIAL_TRANSITORIA_OpeningFcn, ...
                   'gui_OutputFcn',  @SENIAL_TRANSITORIA_OutputFcn, ...
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


% --- Executes just before SENIAL_TRANSITORIA is made visible.
function SENIAL_TRANSITORIA_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to SENIAL_TRANSITORIA (see VARARGIN)

% Choose default command line output for SENIAL_TRANSITORIA
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes SENIAL_TRANSITORIA wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = SENIAL_TRANSITORIA_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in boton_calcularx.
function boton_calcularx_Callback(hObject, eventdata, handles)
% hObject    handle to boton_calcularx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function editor_x_Callback(hObject, eventdata, handles)
% hObject    handle to editor_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editor_x as text
%        str2double(get(hObject,'String')) returns contents of editor_x as a double


% --- Executes during object creation, after setting all properties.
function editor_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editor_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editor_linfx_Callback(hObject, eventdata, handles)
% hObject    handle to editor_linfx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editor_linfx as text
%        str2double(get(hObject,'String')) returns contents of editor_linfx as a double


% --- Executes during object creation, after setting all properties.
function editor_linfx_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editor_linfx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editor_lsupx_Callback(hObject, eventdata, handles)
% hObject    handle to editor_lsupx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editor_lsupx as text
%        str2double(get(hObject,'String')) returns contents of editor_lsupx as a double


% --- Executes during object creation, after setting all properties.
function editor_lsupx_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editor_lsupx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in boton_graficarx.
function boton_graficarx_Callback(hObject, eventdata, handles)
% hObject    handle to boton_graficarx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global esc Lx amptx txi_n txi_f txi_p txi ampx x  xi_n xi_f xi_p xi %se declaran variables globales
syms t %se declaran variables simbolicas
fun=get(handles.menu_x,'value');
axes(handles.graficax);

switch fun
    case 1
    clc %limpia los comandos y posicion anteriores del cursor
    %axes(handles.grafica_x)%establece ejes en graficas_x
    set(handles.graficax, 'visible', 'on')%establece visibles los ejes de la grafica_x
    cla %limpia los ejes 

    esc=100;%establecemos la escala
    
    %Lx(1) = str2num(get(handles.editor_linfx, 'String'));%se obtiene el limite inferior de la funcion x(t)
    %Lx(2) = str2num(get(handles.editor_lsupx, 'String'));%se obtiene el limite superior de la funcion x(t)
    
    %amptx=max(Lx)-min(Lx); %calculamos la amplitud del tiempo 
    %txi_n=(min(Lx)-amptx):1/esc:min(Lx); %valores negativos para el tiempo
    txi=-2*pi:1/esc:2*pi; %valores de tiempo para la funcion
    %txi_p=max(Lx):1/esc:(max(Lx)+amptx);%valores positivos para el tiempo
    %txi=[txi_n,txi_f,txi_p]; %vector del tiempo

    x = sin (t);%se obtiene la ecuacion y se le asigna a x(t)
    %xi_n=zeros(1,amptx*esc+1); %valores negativos para la funcion
    xi =eval(subs(x,txi));%reemplaza los valores de txi_f en la ecuacion x  
    %xi_p=zeros(1,amptx*esc+1); %valores positivos para la funcion
    %xi=[xi_n,xi_f,xi_p]; %vector de la funcion

    ampx=max(xi)-min(xi); %calculamos la amplitud de la funcion

    plot([0 0],[min(xi)-ampx max(xi)+ampx],'black','Linewidth', 1);hold on %grafica del eje y
    plot([txi(1) txi(end)],[0 0],'black','Linewidth', 1);hold on %grafica del eje x
    plot(txi,xi,'blue','Linewidth', 1.5); hold on %grafica la funcion
    xlim([txi(1) txi(end)]) %se define el los limites fijos para el eje x
    ylim([min(xi)-ampx max(xi)+ampx])  %se define el los limites fijos para el eje y
    grid on%pone cuadriculas a la grafica
    xlabel('TIEMPO'); %titulo del eje x
    ylabel('MAGNITUD'); %titulo del eje y
    title('x(t)','FontName','Cambria','FontSize',12); %titulo del grafico

guidata(hObject, handles);    
end





% --- Executes on selection change in menu_x.
function menu_x_Callback(hObject, eventdata, handles)
% hObject    handle to menu_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns menu_x contents as cell array
%        contents{get(hObject,'Value')} returns selected item from menu_x


% --- Executes during object creation, after setting all properties.
function menu_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to menu_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editor_caracteristica3_Callback(hObject, eventdata, handles)
% hObject    handle to editor_caracteristica3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editor_caracteristica3 as text
%        str2double(get(hObject,'String')) returns contents of editor_caracteristica3 as a double


% --- Executes during object creation, after setting all properties.
function editor_caracteristica3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editor_caracteristica3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editor_caracteristica2_Callback(hObject, eventdata, handles)
% hObject    handle to editor_caracteristica2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editor_caracteristica2 as text
%        str2double(get(hObject,'String')) returns contents of editor_caracteristica2 as a double


% --- Executes during object creation, after setting all properties.
function editor_caracteristica2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editor_caracteristica2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in boton_graficar.
function boton_graficar_Callback(hObject, eventdata, handles)
% hObject    handle to boton_graficar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global esc wn A R B C txi ampx x xi %se declaran variables globales
syms t %se declaran variables simbolicas

clc %limpia los comandos y posicion anteriores del cursor
axes(handles.graficax)%establece ejes en graficas_x
set(handles.graficax, 'visible', 'on')%establece visibles los ejes de la grafica_x
cla %limpia los ejes 

wn=str2num(get(handles.editor_fna, 'String'));
A=str2num(get(handles.editor_ca, 'String'));
R=str2num(get(handles.editor_R, 'String'));

esc=10;%establecemos la escala
ts=4/(A*wn);
txi=0:1/esc:ts+ts/2;%[txi_n,txi_f,txi_p]; %vector del tiempo
B=sqrt(1-A*A);
C=atan(B/A);
x = R*(1-exp(-A*wn*t)*sin(wn*B*t+C)/B);%eval(get(handles.editor_xt, 'String'));%se obtiene la ecuacion y se le asigna a x(t)
xi =eval(subs(x,txi));%reemplaza los valores de txi_f en la ecuacion x  

ampx=max(xi)-min(xi); %calculamos la amplitud de la funcion
tp=round(pi/(wn*B),2);
plot([0 0],[min(xi)-ampx/2 max(xi)+ampx/4],'black','Linewidth', 1);hold on %grafica del eje y
plot([txi(1)-ts/4 txi(end)],[0 0],'black','Linewidth', 1);hold on %grafica del eje x
plot(txi,xi,'blue','Linewidth', 1.5); hold on %grafica la funcion
xlim([txi(1)-ts/4 txi(end)]) %se define el los limites fijos para el eje x
ylim([min(xi)-ampx/2 max(xi)+ampx/4])  %se define el los limites fijos para el eje y
grid on%pone cuadriculas a la grafica
xlabel('TIEMPO'); %titulo del eje x
ylabel('MAGNITUD'); %titulo del eje y
title('x(t)','FontName','Cambria','FontSize',12); %titulo del grafico

guidata(hObject, handles);

function editor_fna_Callback(hObject, eventdata, handles)
% hObject    handle to editor_fna (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editor_fna as text
%        str2double(get(hObject,'String')) returns contents of editor_fna as a double


% --- Executes during object creation, after setting all properties.
function editor_fna_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editor_fna (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editor_ca_Callback(hObject, eventdata, handles)
% hObject    handle to editor_ca (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editor_ca as text
%        str2double(get(hObject,'String')) returns contents of editor_ca as a double


% --- Executes during object creation, after setting all properties.
function editor_ca_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editor_ca (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in boton_calcular.
function boton_calcular_Callback(hObject, eventdata, handles)
% hObject    handle to boton_calcular (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global esc wn A R B C txi ampx x xi %se declaran variables globales
syms t %se declaran variables simbolicas

clc %limpia los comandos y posicion anteriores del cursor
axes(handles.graficax)%establece ejes en graficas_x
set(handles.graficax, 'visible', 'on')%establece visibles los ejes de la grafica_x
x
ts=round(4/(A*wn),2)
tp=round(pi/(wn*B),2)
xmax =eval(subs(x,tp))
xinf =eval(subs(x,ts))
SP=round(xmax-xinf,2);
td=round(0.4*tp,2);
t1=round(0.1*tp,2);
t2=round(0.7*tp,2);
xd=diff(0.5*xinf-x);
x1=diff(0.1*xinf-x);
x2=diff(0.9*xinf-x);
for i=1:25
    tdi(i)=td-eval(subs(0.5*xinf-x,td))/eval(subs(xd,td));
    td=tdi(i);
end
for j=1:25
    t1i(j)=t1-eval(subs(0.1*xinf-x,t1))/eval(subs(x1,t1));
    t1=t1i(j);
end
for k=1:25
    t2i(k)=t2-eval(subs(0.9*xinf-x,t2))/eval(subs(x2,t2));
    t2=t2i(k);
end
td=round(tdi(end),2)
tr=round(t2i(end)-t1i(end),2)
str=num2str(SP);
set(handles.texto_Sp,'string',str)
str=num2str(ts);
set(handles.texto_ts,'string',str)
str=num2str(td);
set(handles.texto_td,'string',str)
str=num2str(tr);
set(handles.texto_tr,'string',str)
str=num2str(tp);
set(handles.texto_tp,'string',str)

plot([0 0],[xinf xmax],'-kx','markerfacecolor','magenta','LineWidt',1.5) %grafica el intervalo para SP
plot([0 tp],[xmax xmax],':k','markerfacecolor','magenta','LineWidt',1)
text(-t2/2,xinf+SP/2,'Sp','FontName','Cambria','FontSize',15,'color','magenta')
plot([tp tp],[xmax 0],':ko','markerfacecolor','cyan','LineWidt',1) %grafica de tp
text(tp,-ampx/8,'tp','FontName','Cambria','FontSize',15,'color','cyan')

plot([ts ts],[xinf 0],':ko','markerfacecolor','red','LineWidt',1) %grafica el tiempo de asentamiento
text(ts,-ampx/8,'ts','FontName','Cambria','FontSize',15,'color','red') %grafica el texto del tiempo asentamiento
plot([t2 t2],[0.9*xinf 0],':ko','markerfacecolor','black','LineWidt',1) %grafica t2
text(t2,-ampx/8,'t2','FontName','Cambria','FontSize',15) %grafica el texto t2
plot([t1 t1],[0.1*xinf 0],':ko','markerfacecolor','black','LineWidt',1) %grafica t1
text(t1,-ampx/8,'t1','FontName','Cambria','FontSize',15) %grafica el texto t1
plot([t1 t2],[-ampx/4 -ampx/4],':kx','markerfacecolor','black','LineWidt',1) %grafica linea tr
text((t1+t2)/2,-ampx/3,'tr','FontName','Cambria','FontSize',15) %grafica el texto tr
plot([td td],[0.5*xinf 0],':ko','markerfacecolor','green','LineWidt',1) %grafica td
text(td,-ampx/8,'td','FontName','Cambria','FontSize',15,'color','green') %grafica el texto td
plot([0 ts+ts/2],[xinf xinf],':k','markerfacecolor','black','LineWidt',1) %graficamos xinf
guidata(hObject, handles);



function editor_R_Callback(hObject, eventdata, handles)
% hObject    handle to editor_R (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editor_R as text
%        str2double(get(hObject,'String')) returns contents of editor_R as a double


% --- Executes during object creation, after setting all properties.
function editor_R_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editor_R (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit39_Callback(hObject, eventdata, handles)
% hObject    handle to edit39 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit39 as text
%        str2double(get(hObject,'String')) returns contents of edit39 as a double


% --- Executes during object creation, after setting all properties.
function edit39_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit39 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit38_Callback(hObject, eventdata, handles)
% hObject    handle to edit38 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit38 as text
%        str2double(get(hObject,'String')) returns contents of edit38 as a double


% --- Executes during object creation, after setting all properties.
function edit38_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit38 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit37_Callback(hObject, eventdata, handles)
% hObject    handle to edit37 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit37 as text
%        str2double(get(hObject,'String')) returns contents of edit37 as a double


% --- Executes during object creation, after setting all properties.
function edit37_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit37 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit36_Callback(hObject, eventdata, handles)
% hObject    handle to edit36 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit36 as text
%        str2double(get(hObject,'String')) returns contents of edit36 as a double


% --- Executes during object creation, after setting all properties.
function edit36_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit36 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global esc AS BS CS DS txi ampx x xi %se declaran variables globales
syms t %se declaran variables simbolicas

clc %limpia los comandos y posicion anteriores del cursor
axes(handles.graficax)%establece ejes en graficas_x
set(handles.graficax, 'visible', 'on')%establece visibles los ejes de la grafica_x
cla %limpia los ejes 

AS=str2num(get(handles.edit36, 'String'));
BS=str2num(get(handles.edit37, 'String'));
CS=str2num(get(handles.edit38, 'String'));
DS=str2num(get(handles.edit39, 'String'));

esc=10;%establecemos la escala
ts=4*CS; %se obtiene el tiempo de establecimiento
x = AS-BS*exp(-t/CS).*cos(DS*t);%eval(get(handles.editor_xt, 'String'));%se obtiene la ecuacion y se le asigna a x(t)
tp=round(atan(-CS/DS)/DS+pi/DS,2); %calcula el tiempo de pico
xinf=eval(subs(x,ts)); %calcula el valor cuando el tiempo llega al infinito
td=round(0.4*tp,2); %valor inicial de td 
t1=round(0.1*tp,2); %valor inicial de t1
xd=diff(0.5*xinf-x); %derivada respecto a td
x1=diff(0.1*xinf-x); %derivada respecto a t1
for i=1:25
    tdi(i)=td-eval(subs(0.5*xinf-x,td))/eval(subs(xd,td));
    td=tdi(i);
end
for j=1:25
    t1i(j)=t1-eval(subs(0.1*xinf-x,t1))/eval(subs(x1,t1));
    t1=t1i(j);
end
txi=t1i(end)-ts/8:1/esc:1.1*ts+0.5/tdi(end);%vector del tiempo

xi =eval(subs(x,txi));%reemplaza los valores de txi_f en la ecuacion x  

ampx=max(xi)-min(xi); %calculamos la amplitud de la funcion

plot([0 0],[min(xi)-ampx/4 max(xi)+ampx/4],'black','Linewidth', 1);hold on %grafica del eje y
plot([txi(1) txi(end)],[0 0],'black','Linewidth', 1);hold on %grafica del eje x
plot(txi,xi,'blue','Linewidth', 1.5); hold on %grafica la funcion
xlim([txi(1) txi(end)]) %se define el los limites fijos para el eje x
ylim([min(xi)-ampx/8 max(xi)+ampx/8])  %se define el los limites fijos para el eje y
grid on %pone cuadriculas a la grafica
xlabel('TIEMPO'); %titulo del eje x
ylabel('MAGNITUD'); %titulo del eje y
title('x(t)','FontName','Cambria','FontSize',12); %titulo del grafico

guidata(hObject, handles);

% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global esc AS BS CS DS ampx x xi %se declaran variables globales
syms t %se declaran variables simbolicas

clc %limpia los comandos y posicion anteriores del cursor
axes(handles.graficax)%establece ejes en graficas_x
set(handles.graficax, 'visible', 'on')%establece visibles los ejes de la grafica_x
ts=round(4*CS,2);
tp=round(atan(-CS/DS)/DS+pi/DS,2);
xmax =eval(subs(x,tp))
xinf =eval(subs(x,ts));%eval(subs(x,ts))%reemplaza los valores de txi_f en la ecuacion x  
SP=round(xmax-xinf,2);
td=0; %round(0.4*tp,2);
t1=0; %round(0.1*tp,2);
t2=0; %round(0.7*tp,2);
xd=diff(0.5*xinf-x);
x1=diff(0.1*xinf-x);
x2=diff(0.9*xinf-x);
for i=1:25
    tdi(i)=td-eval(subs(0.5*xinf-x,td))/eval(subs(xd,td));
    td=tdi(i);
end
for j=1:25
    t1i(j)=t1-eval(subs(0.1*xinf-x,t1))/eval(subs(x1,t1));
    t1=t1i(j);
end
for k=1:25
    t2i(k)=t2-eval(subs(0.9*xinf-x,t2))/eval(subs(x2,t2));
    t2=t2i(k);
end
t2i(end)
t1i(end)
tr=round(t2i(end)-t1i(end),2);
td=round(tdi(end),2);
str=num2str(SP);
set(handles.texto_Sp,'string',str)
str=num2str(ts);
set(handles.texto_ts,'string',str)
str=num2str(td);
set(handles.texto_td,'string',str)
str=num2str(tr);
set(handles.texto_tr,'string',str)
str=num2str(tp);
set(handles.texto_tp,'string',str)

plot([0 0],[xinf xmax],'-kx','markerfacecolor','black','LineWidt',1.5) %grafica el intervalo para SP
plot([0 tp],[xmax xmax],':k','markerfacecolor','black','LineWidt',1)
text(t1-ts/8,xinf+SP/2,'Sp','FontName','Cambria','FontSize',15,'color','magenta')
plot([tp tp],[xmax 0],':ko','markerfacecolor','cyan','LineWidt',1) %grafica de tp
text(tp,-ampx/8,'tp','FontName','Cambria','FontSize',15,'color','cyan')

plot([ts ts],[xinf 0],':ko','markerfacecolor','red','LineWidt',1) %grafica el tiempo de asentamiento
text(ts,-ampx/8,'ts','FontName','Cambria','FontSize',15,'color','red') %grafica el texto del tiempo asentamiento
plot([t2 t2],[0.9*xinf 0],':ko','markerfacecolor','black','LineWidt',1) %grafica t2
text(t2,-ampx/8,'t2','FontName','Cambria','FontSize',15) %grafica el texto t2
plot([t1 t1],[0.1*xinf 0],':ko','markerfacecolor','black','LineWidt',1) %grafica t1
text(t1,-ampx/8,'t1','FontName','Cambria','FontSize',15) %grafica el texto t1
plot([t1 t2],[-ampx/4 -ampx/4],':kx','markerfacecolor','black','LineWidt',1) %grafica linea tr
text((t1+t2)/2,-ampx/3,'tr','FontName','Cambria','FontSize',15) %grafica el texto tr
plot([td td],[0.5*xinf 0],':ko','markerfacecolor','green','LineWidt',1) %grafica td
text(td,-ampx/8,'td','FontName','Cambria','FontSize',15,'color','green') %grafica el texto td
plot([0 ts+ts/2],[xinf xinf],':k','markerfacecolor','black','LineWidt',1) %graficamos xinf
text(0,0.9*xinf,'x(inf)','FontName','Cambria','FontSize',15)
guidata(hObject, handles);

% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global esc AP BP CP ampx x xi %se declaran variables globales
syms t %se declaran variables simbolicas

clc %limpia los comandos y posicion anteriores del cursor
axes(handles.graficax)%establece ejes en graficas_x
set(handles.graficax, 'visible', 'on')%establece visibles los ejes de la grafica_x

t1=round(-CP*log((AP*0.1-AP)/-BP),2);%se obtiene el tiempo t1 10%
t2=round(-CP*log((AP*0.9-AP)/-BP),2);%se obtiene el tiempo t2 90%
tr=round(t2-t1,2); %se obtiene el tiempo de levantamiento tr
ts=4*CP; %se obtiene el tiempo de asentamiento ts
xmax =eval(subs(x,inf)); %se obtiene el valor maximo
xinf =eval(subs(x,inf)); %se obtiene el valor cuando t tiende al infinito
SP=xmax-xinf; %se obtiene el maximo sobrepaso 
td=round(-CP*log((AP*0.5-AP)/-BP),2); %se obtiene el tiempo de retraso td
xs =eval(subs(x,ts));
xmax =eval(subs(x,inf));

str=num2str(SP);
set(handles.texto_Sp,'string',str)
str=num2str(ts);
set(handles.texto_ts,'string',str)
str=num2str(td);
set(handles.texto_td,'string',str)
str=num2str(tr);
set(handles.texto_tr,'string',str)
str=num2str(inf);
set(handles.texto_tp,'string',str)

plot([ts ts],[xs 0],':ko','markerfacecolor','red','LineWidt',1) %grafica el tiempo de asentamiento
text(ts,-ampx/8,'ts','FontName','Cambria','FontSize',15,'color','red') %grafica el texto del tiempo asentamiento
plot([t2 t2],[0.9*xinf 0],':ko','markerfacecolor','black','LineWidt',1) %grafica t2
text(t2,-ampx/8,'t2','FontName','Cambria','FontSize',15) %grafica el texto t2
plot([t1 t1],[0.1*xinf 0],':ko','markerfacecolor','black','LineWidt',1) %grafica t1
text(t1,-ampx/8,'t1','FontName','Cambria','FontSize',15) %grafica el texto t1
plot([t1 t2],[-ampx/4 -ampx/4],':kx','markerfacecolor','black','LineWidt',1) %grafica linea tr
text((t1+t2)/2,-ampx/3,'tr','FontName','Cambria','FontSize',15) %grafica el texto tr
plot([td td],[0.5*xinf 0],':ko','markerfacecolor','green','LineWidt',1) %grafica td
text(td,-ampx/8,'td','FontName','Cambria','FontSize',15,'color','green') %grafica el texto td
plot([0 ts+ts/2],[xinf xinf],':k','markerfacecolor','black','LineWidt',1) %graficamos xinf
text(0,0.9*xinf,'x(inf)','FontName','Cambria','FontSize',15)
guidata(hObject, handles);

% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global esc AP BP CP txi ampx x xi %se declaran variables globales
syms t %se declaran variables simbolicas

clc %limpia los comandos y posicion anteriores del cursor
axes(handles.graficax)%establece ejes en graficas_x
set(handles.graficax, 'visible', 'on')%establece visibles los ejes de la grafica_x
cla %limpia los ejes 

AP=str2num(get(handles.edit24, 'String')); %se obtiene el valor de A
BP=str2num(get(handles.edit27, 'String')); %se obtiene el valor de B
CP=str2num(get(handles.edit28, 'String')); %se obtiene el valor de C

esc=10;%establecemos la escala
ts=4*CP; %tiempo de establecimiento
t1=-CP*log((AP*0.1-AP)/-BP),2;
td=-CP*log((AP*0.5-AP)/-BP),2;
txi=t1-ts/8:1/esc:1.1*ts+0.5/td; %vector del tiempo

x = AP-BP*exp(-t/CP); %ecuación del sistema de primer orden
xi =eval(subs(x,txi)); %reemplaza los valores de txi en la ecuacion x  

ampx=max(xi)-min(xi); %calculamos la amplitud de la funcion

plot([0 0],[min(xi)-ampx/4 max(xi)+ampx/4],'black','Linewidth', 1);hold on %grafica del eje y
plot([txi(1) txi(end)],[0 0],'black','Linewidth', 1);hold on %grafica del eje x
plot(txi,xi,'blue','Linewidth', 1.5); hold on %grafica la funcion
xlim([txi(1) txi(end)]) %se define el los limites fijos para el eje x
ylim([min(xi)-ampx/8 max(xi)+ampx/8])  %se define el los limites fijos para el eje y
grid on %pone cuadriculas a la grafica
xlabel('TIEMPO'); %titulo del eje x
ylabel('MAGNITUD'); %titulo del eje y
title('x(t)','FontName','Cambria','FontSize',12); %titulo del grafico

guidata(hObject, handles);

function edit24_Callback(hObject, eventdata, handles)
% hObject    handle to edit24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit24 as text
%        str2double(get(hObject,'String')) returns contents of edit24 as a double


% --- Executes during object creation, after setting all properties.
function edit24_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit27_Callback(hObject, eventdata, handles)
% hObject    handle to edit27 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit27 as text
%        str2double(get(hObject,'String')) returns contents of edit27 as a double


% --- Executes during object creation, after setting all properties.
function edit27_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit27 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit28_Callback(hObject, eventdata, handles)
% hObject    handle to edit28 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit28 as text
%        str2double(get(hObject,'String')) returns contents of edit28 as a double


% --- Executes during object creation, after setting all properties.
function edit28_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit28 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
