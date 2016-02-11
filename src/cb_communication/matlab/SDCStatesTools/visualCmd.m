function varargout = visualCmd(varargin)
% VISUALCMD M-file for visualCmd.fig
%      VISUALCMD, by itself, creates a new VISUALCMD or raises the existing
%      singleton*.
%
%      H = VISUALCMD returns the handle to a new VISUALCMD or the handle to
%      the existing singleton*.
%
%      VISUALCMD('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in VISUALCMD.M with the given input arguments.
%
%      VISUALCMD('Property','Value',...) creates a new VISUALCMD or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before visualCmd_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to visualCmd_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help visualCmd

% Last Modified by GUIDE v2.5 12-Sep-2009 20:12:04

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @visualCmd_OpeningFcn, ...
                   'gui_OutputFcn',  @visualCmd_OutputFcn, ...
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


% --- Executes just before visualCmd is made visible.
function visualCmd_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to visualCmd (see VARARGIN)

% Choose default command line output for visualCmd
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes visualCmd wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = visualCmd_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in showAllButton.
function showAllButton_Callback(hObject, eventdata, handles)
% hObject    handle to showAllButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in showControlButton.
function showControlButton_Callback(hObject, eventdata, handles)
% hObject    handle to showControlButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in showStatusButton.
function showStatusButton_Callback(hObject, eventdata, handles)
% hObject    handle to showStatusButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in hideStatusButton.
function hideStatusButton_Callback(hObject, eventdata, handles)
% hObject    handle to hideStatusButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in hideControlButton.
function hideControlButton_Callback(hObject, eventdata, handles)
% hObject    handle to hideControlButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in hideAllButton.
function hideAllButton_Callback(hObject, eventdata, handles)
% hObject    handle to hideAllButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


