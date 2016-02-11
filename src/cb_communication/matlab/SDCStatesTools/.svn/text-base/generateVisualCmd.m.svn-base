function generateVisualCmd(cmdData, jointNames, closeHandle)
  
  vis_hd = visualCmd;
  children_hd = get(vis_hd,'children');
  
  
  for i = 1:length(children_hd)
    if(strcmp(get(children_hd(i),'tag'),'dataCmdTable'))
      dataCmd_hd = children_hd(i);
    end
    if(strcmp(get(children_hd(i),'tag'),'controlBitsPanel'))
      controlBitsSelect_hd = children_hd(i);
    end
    if(strcmp(get(children_hd(i),'tag'),'statusBitsPanel'))
      statusBitsSelect_hd = children_hd(i);
    end
    if(strcmp(get(children_hd(i),'tag'),'dofSelect'))
      dofSelect_hd = children_hd(i);
    end
    if(strcmp(get(children_hd(i),'tag'),'showStatusButton'))
      showStatus_hd = children_hd(i);
    end
    if(strcmp(get(children_hd(i),'tag'),'hideStatusButton'))
      hideStatus_hd = children_hd(i);
    end
    if(strcmp(get(children_hd(i),'tag'),'showControlButton'))
      showControl_hd = children_hd(i);
    end
    if(strcmp(get(children_hd(i),'tag'),'hideControlButton'))
      hideControl_hd = children_hd(i);
    end
    if(strcmp(get(children_hd(i),'tag'),'showAllButton'))
      showAll_hd = children_hd(i);
    end
    if(strcmp(get(children_hd(i),'tag'),'hideAllButton'))
      hideAll_hd = children_hd(i);
    end
  end
  
  set(vis_hd,'deleteFcn',{closeHandle,dataCmd_hd});
  
  %%we read the name of the parameters bits
  fd = fopen('cmdBits.conf','r');
  if(fd==-1)
    fprintf('ERROR opening param map file\n');
    return;
  end
 
  ind = 1;
  paramIndex = [];
  
  while(~feof(fd))
    paramNames{ind} = fscanf(fd,'%s',1);
    paramIndex(ind,:) = fscanf(fd,'%d',[2 1]);
    paramType{ind} = fscanf(fd,'%s',1);
    ind = ind+1;
  end
 
  numParams = length(paramNames);
  
  controlIndexes = [];
  statusIndexes = [];
  for i = 1:length(paramNames)
    if(strcmp(paramType(i),'control'))
      controlIndexes = [controlIndexes i];
    elseif(strcmp(paramType(i),'status'))
      statusIndexes = [statusIndexes i];
    end
  end
  
  displayedJointIndex =  1:length(jointNames);
  displayedParamIndex = sort([controlIndexes statusIndexes]);
  
   
  %%%we create the check list for the parameters
  t1 = get(controlBitsSelect_hd,'position');
  t2 = get(statusBitsSelect_hd,'position');
  select_width1 = t1(3);
  select_height1 = t1(4);
  
  select_width2 = t2(3);
  select_height2 = t2(4);
  
  x1 = 1;
  y1 = select_height1-40;
  w1 = 300;
  h1 = 20;
  
  x2 = 1;
  y2 = select_height2-40;
  w2 = 300;
  h2 = 20;
  
  for i = 1:length(paramNames)
    if(strcmp(paramType(i),'control'))
      chkBoxParams_hd(i) = uicontrol(controlBitsSelect_hd,'style','checkbox','string',paramNames(i),'Position',[x1 y1 w1 h1],'value',1,'tag',paramNames{i});
    
      y1 = y1-h1;
      if(y1<0)
        x1 = x1 + w1;
        y1 = select_height1-40;
      end
    elseif(strcmp(paramType(i),'status'))
      chkBoxParams_hd(i) = uicontrol(statusBitsSelect_hd,'style','checkbox','string',paramNames(i),'Position',[x2 y2 w2 h2],'value',1,'tag',paramNames{i});
    
      y2 = y2-h2;
      if(y2<0)
        x2 = x2 + w2;
        y2 = select_height1-40;
      end
    else
      chkBoxParams_hd(i) = -1;
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
  
  
  %%we create the data from the numeric to bits  
  dataCmd = zeros(length(jointNames),length(paramNames));
  for i = 1:length(jointNames)
    str = strcat(fliplr(dec2bin(cmdData(i,1),8)),fliplr(dec2bin(cmdData(i,2),8)),fliplr(dec2bin(cmdData(i,3),8)));
    for j = 1:length(paramNames)
      dataCmd(i,j) = str2num(str(j));
    end
  end
  
  %%to be shared with the calling GUI
  set(dataCmd_hd,'userData',dataCmd);
  
  
  set(dataCmd_hd,'data',dataCmd(displayedJointIndex,displayedParamIndex));
  set(dataCmd_hd,'ColumnName',paramNames);
  set(dataCmd_hd,'RowName',jointNames);
  
    
  %%%%set the callbacks%%%%%%%%%%%%%%%%%
  for i = 1:length(paramNames)
    if(chkBoxParams_hd(i)~=-1)
      set(chkBoxParams_hd(i),'callback',{@updateView});
    end
  end
  for i = 1:length(jointNames)
    set(chkBoxJoints_hd(i),'callback',{@updateView});
  end
  
  set(dataCmd_hd,'cellEditCallback',{@updateDataTable});
  set(showStatus_hd,'callback',{@setAllParams,chkBoxParams_hd(statusIndexes),1});
  set(hideStatus_hd,'callback',{@setAllParams,chkBoxParams_hd(statusIndexes),0});
  set(showControl_hd,'callback',{@setAllParams,chkBoxParams_hd(controlIndexes),1});
  set(hideControl_hd,'callback',{@setAllParams,chkBoxParams_hd(controlIndexes),0});
  set(showAll_hd,'callback',{@setAllParams,chkBoxParams_hd,1});
  set(hideAll_hd,'callback',{@setAllParams,chkBoxParams_hd,0});


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function updateDataTable(hObject,eventData)
%this function reads the current values set in the table and
%update the rawDataStructure accordingly
    
    dataCmd(displayedJointIndex, displayedParamIndex) = get(dataCmd_hd,'data');

    set(dataCmd_hd,'userData',dataCmd);
  end
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
function updateView(hObject,eventData)
  %this function checks the state of all checkbox elements and
  %update the table view accordingly
 
  
  displayedParamIndex = [];
  displayedJointIndex = [];
  
  for i = 1:length(paramNames)
    if(chkBoxParams_hd(i)~=-1)
      if(get(chkBoxParams_hd(i),'value')==1)
        displayedParamIndex = [displayedParamIndex i];
      end
    end
  end
  
  for i = 1:length(jointNames)
    if(get(chkBoxJoints_hd(i),'value')==1)
      displayedJointIndex = [displayedJointIndex i];
    end
  end
 
  
  set(dataCmd_hd,'data',dataCmd(displayedJointIndex,displayedParamIndex));
  set(dataCmd_hd,'ColumnName',paramNames(displayedParamIndex));
  set(dataCmd_hd,'RowName',jointNames(displayedJointIndex));
  
  end
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
function setAllParams(hObject,eventData,option_hd,value)
%this function set all the check_box option_hd to value (0,1)
  
  for i = 1:length(option_hd)
    if(option_hd(i)~=-1)
      set(option_hd(i),'value',value);
    end
  end
  
  updateView;
  
  end
  
  
  end
  