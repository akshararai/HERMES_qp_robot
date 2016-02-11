function varargout = visual1(varargin)
% VISUAL1 M-file for visual1.fig
%      VISUAL1, by itself, creates a new VISUAL1 or raises the existing
%      singleton*.
%
%      H = VISUAL1 returns the handle to a new VISUAL1 or the handle to
%      the existing singleton*.
%
%      VISUAL1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in VISUAL1.M with the given input arguments.
%
%      VISUAL1('Property','Value',...) creates a new VISUAL1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before visual1_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to visual1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help visual1

% Last Modified by GUIDE v2.5 12-Sep-2009 17:44:20

% Begin initialization code - DO NOT EDIT
gui_Singleton = 0;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @visual1_OpeningFcn, ...
                   'gui_OutputFcn',  @visual1_OutputFcn, ...
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


% --- Executes just before visual1 is made visible.
function visual1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to visual1 (see VARARGIN)

% Choose default command line output for visual1
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes visual1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = visual1_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in viewAllParamsButton.
function viewAllParamsButton_Callback(hObject, eventdata, handles)
% hObject    handle to viewAllParamsButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in hideAllParamsButton.
function hideAllParamsButton_Callback(hObject, eventdata, handles)
% hObject    handle to hideAllParamsButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in viewAllJointsButton.
function viewAllJointsButton_Callback(hObject, eventdata, handles)
% hObject    handle to viewAllJointsButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in hideAllJointsButton.
function hideAllJointsButton_Callback(hObject, eventdata, handles)
% hObject    handle to hideAllJointsButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in saveDataButton.
function saveDataButton_Callback(hObject, eventdata, handles)
% hObject    handle to saveDataButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function saveFileName_Callback(hObject, eventdata, handles)
% hObject    handle to saveFileName (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of saveFileName as text
%        str2double(get(hObject,'String')) returns contents of saveFileName as a double


% --- Executes during object creation, after setting all properties.
function saveFileName_CreateFcn(hObject, eventdata, handles)
% hObject    handle to saveFileName (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in showCommandBitsButton.
function showCommandBitsButton_Callback(hObject, eventdata, handles)
% hObject    handle to showCommandBitsButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
delete(hObject);


