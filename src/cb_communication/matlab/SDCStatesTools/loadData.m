function [paramNames jointNames jointIndex rawDataTable] = loadData(paramsMapFileName, jointMapFileName, path, nameRoot)
  
 
  for i = 1:5
    fileNames{i} = sprintf('%s_192.168.%d.0',nameRoot,i);
  end
  
  
  %%we read the parameters ordering
  %%we load the list of parameters that are to be displayed
  fd = fopen(paramsMapFileName,'r');
  if(fd==-1)
    fprintf('ERROR opening param map file\n');
    return;
  end
 
  ind = 1;
  while(~feof(fd))
    paramNames{ind} = fscanf(fd,'%s',1);
     ind = ind+1;
  end
  numParams = length(paramNames);
  
  
  %%we read the joint mapping
  fd = fopen(jointMapFileName,'r');
  if(fd==-1)
    fprintf('ERROR opening joint map file\n');
    return;
  end
  
  ind = 1;
  jointIndex = [];
  while(~feof(fd))
    jointNames{ind} = fscanf(fd,'%s',1);
    jointIndex(ind,:) = fscanf(fd,'%d',[2 1]);
    ind = ind+1;
  end
  numJoints = length(jointIndex);
  
  
  %%%we create the initial data table
  rawDataTable = zeros(numJoints,numParams);
  
  cardIndex = getIndexesFromName('Card_number',paramNames);
  
  
  for i = 1:length(fileNames)
    file = fopen(fullfile(path,fileNames{i}),'r');
    if(file==-1)
        fprintf('ERROR opening file %s\n',fullfile(path,fileNames{i}));
    end
    while(~feof(file))
      tmp = fscanf(file, 'Card%d');
      j = find((jointIndex(:,1) == i) & (jointIndex(:,2)==tmp));
      rawDataTable(j,cardIndex) = jointIndex(j,2);
      rawDataTable(j,2:numParams) = fscanf(file, '%i', [numParams-1 1]);
      fscanf(file,'\n');
    end  
  end
  
  
  