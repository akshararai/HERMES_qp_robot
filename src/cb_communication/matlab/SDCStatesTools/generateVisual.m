function vis_hd = generateVisual(paramsMapFileName,writeParamsMapFileName,jointMapFileName,path,nameRoot)
  %%%init function for the visulaisation of the options
  %%the nested functions will use the data created here
  
  
  %%first we load the data
  [paramNames jointNames jointIndex rawDataTable] = loadData(paramsMapFileName, jointMapFileName, path, nameRoot);
  
  displayedParamIndex = 1:length(paramNames);
  displayedJointIndex = 1:length(jointNames);
  
  %%we create the visual and get pointers on it and its children
  vis_hd = visual1;
  children_hd = get(vis_hd,'children');
  
  
  for i = 1:length(children_hd)
    if(strcmp(get(children_hd(i),'tag'),'dataTable'))
      data_hd = children_hd(i);
    end
    if(strcmp(get(children_hd(i),'tag'),'dataSelectPanel'))
      dataSelect_hd = children_hd(i);
    end
    if(strcmp(get(children_hd(i),'tag'),'dofSelectPanel'))
      dofSelect_hd = children_hd(i);
    end
    if(strcmp(get(children_hd(i),'tag'),'viewAllParamsButton'))
      viewAllParams_hd = children_hd(i);
    end
    if(strcmp(get(children_hd(i),'tag'),'hideAllParamsButton'))
      hideAllParams_hd = children_hd(i);
    end
    if(strcmp(get(children_hd(i),'tag'),'viewAllJointsButton'))
      viewAllJoints_hd = children_hd(i);
    end
    if(strcmp(get(children_hd(i),'tag'),'hideAllJointsButton'))
      hideAllJoints_hd = children_hd(i);
    end
    if(strcmp(get(children_hd(i),'tag'),'saveDataButton'))
      saveData_hd = children_hd(i);
    end
     if(strcmp(get(children_hd(i),'tag'),'saveFileName'))
      saveName_hd = children_hd(i);
     end
     if(strcmp(get(children_hd(i),'tag'),'showCommandBitsButton'));
       showCommandBits_hd = children_hd(i);
     end
  end
  
  
  %%we set the table to the data loaded previously and the names of
  %the params/joints
  set(data_hd,'data',rawDataTable);
  set(data_hd,'ColumnName',paramNames);
  set(data_hd,'RowName',jointNames);
  
  set(saveName_hd,'string','initialSDCState');
  
  %%%we create the check list for the parameters
  t = get(dataSelect_hd,'position');
  select_width = t(3);
  select_height = t(4);
  
  x = 1;
  y = select_height-40;
  w = 145;
  h = 20;
  
  for i = 1:length(paramNames)
    chkBoxParams_hd(i) = uicontrol(dataSelect_hd,'style','checkbox','string',paramNames(i),'Position',[x y w h],'value',1,'tag',paramNames{i});
    
    y = y-h;
    if(y<0)
      x = x + w;
      y = select_height-40;
    end
    
  end
  
  
  %%%we create the check list for the joints
  t = get(dofSelect_hd,'position');
  select_width = t(3);
  select_height = t(4);
  
  x = 1;
  y = select_height-40;
  w = 100;
  h = 20;
  
  for i = 1:length(jointNames)
    chkBoxJoints_hd(i) = uicontrol(dofSelect_hd,'style','checkbox','string',jointNames(i),'Position',[x y w h],'value',1,'tag',jointNames{i});
    
    y = y-h;
    if(y<0)
      x = x + w;
      y = select_height-40;
    end
    
  end
  

  %%%%set the callbacks%%%%%%%%%%%%%%%%%
  for i = 1:length(paramNames)
    set(chkBoxParams_hd(i),'callback',{@updateView});
  end
  for i = 1:length(jointNames)
    set(chkBoxJoints_hd(i),'callback',{@updateView});
  end
  
  set(viewAllParams_hd,'callback',{@setAllParams,chkBoxParams_hd,1});
  set(hideAllParams_hd,'callback',{@setAllParams,chkBoxParams_hd,0});
  set(viewAllJoints_hd,'callback',{@setAllParams,chkBoxJoints_hd,1});
  set(hideAllJoints_hd,'callback',{@setAllParams,chkBoxJoints_hd,0});
  
  set(data_hd,'cellEditCallback',{@updateDataTable});
 
  set(saveData_hd,'callback',{@saveData_callback});
  
  set(showCommandBits_hd,'callback',{@showCommandBits_callback});
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function showCommandBits_callback(hObject, eventData)
  
  ind0 = getIndexesFromName('Cmd_byte_0',paramNames);
  ind1 = getIndexesFromName('Cmd_byte_1',paramNames);
  ind2 = getIndexesFromName('Cmd_byte_2',paramNames);
  
  cmdData = rawDataTable(:,[ind0 ind1 ind2]);
  generateVisualCmd(cmdData,jointNames,@closeCommandBitsView);

  end
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function closeCommandBitsView(hObject, eventData, data_hd)
  
  ind0 = getIndexesFromName('Cmd_byte_0',paramNames);
  ind1 = getIndexesFromName('Cmd_byte_1',paramNames);
  ind2 = getIndexesFromName('Cmd_byte_2',paramNames);
  
  cmdData = get(data_hd,'userData');
  
  data_cmd = zeros(length(jointNames),3);
  
  for i = 1:8
    for j = 1:3
      data_cmd(:,j) = data_cmd(:,j) + (2^(i-1))*cmdData(:,8*(j-1)+i);
    end
  end
  
  rawDataTable(:,[ind0 ind1 ind2]) = data_cmd;
  
  updateView;
  
  end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
function saveData_callback(hObject, eventData)
  
  name = get(saveName_hd,'string');
  
  saveData(rawDataTable,paramNames,jointIndex,path,name,writeParamsMapFileName);
  
  
  end
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function updateDataTable(hObject,eventData)
  %this function reads the current values set in the table and
  %update the rawDataStructure accordingly
    
  rawDataTable(displayedJointIndex, displayedParamIndex) = get(data_hd,'data');
    
  end
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
function setAllParams(hObject,eventData,option_hd,value)
  %this function set all the check_box option_hd to value (0,1)
  
  for i = 1:length(option_hd)
    set(option_hd(i),'value',value);
  end
  
  updateView;
  
  end
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
function updateView(hObject,eventData)
  %this function checks the state of all checkbox elements and
  %update the table view accordingly
 
  
  displayedParamIndex = [];
  displayedJointIndex = [];
  
  for i = 1:length(paramNames)
    if(get(chkBoxParams_hd(i),'value')==1)
      displayedParamIndex = [displayedParamIndex i];
    end
  end
  
  for i = 1:length(jointNames)
    if(get(chkBoxJoints_hd(i),'value')==1)
      displayedJointIndex = [displayedJointIndex i];
    end
  end
 
  
  set(data_hd,'data',rawDataTable(displayedJointIndex,displayedParamIndex));
  set(data_hd,'ColumnName',paramNames(displayedParamIndex));
  set(data_hd,'RowName',jointNames(displayedJointIndex));
  
  end
  
  
  
  end