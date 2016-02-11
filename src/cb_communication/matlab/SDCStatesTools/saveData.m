function saveData(dataSet,paramNames, jointIndex, path, nameRoot, writeParamsMapFileName)
  
 
  for i = 1:5
    fileNames{i} = sprintf('%s_192.168.%d.0',nameRoot,i);
  end
 
  %cardNum = [7,7,7,9,8];
  
  %%we read the params to save
  fd = fopen(writeParamsMapFileName,'r');
  if(fd==-1)
    fprintf('ERROR opening write param map file %s\n',writeParamsMapFileName);
    return;
  end
  
  ind = 1;
  while(~feof(fd))
    writeParamNames{ind} = fscanf(fd,'%s',1);
    ind = ind+1;
  end
  numParams = length(writeParamNames);
  
  %%we construct the index to map dataSet
  indexes = zeros(1,numParams);
  for i = 1:numParams
    indexes(i) = getIndexesFromName(writeParamNames{i},paramNames);
  end
  
  %%we get the data to save
  savedData = dataSet(:,indexes);
  
  ind0 = getIndexesFromName('Cmd_byte_0',writeParamNames);
  ind1 = getIndexesFromName('Cmd_byte_1',writeParamNames);
  ind2 = getIndexesFromName('Cmd_byte_2',writeParamNames);
  
  cardNum = getIndexesFromName('Card_number',writeParamNames);
  
  %%we save the data in one file per network
  
  for i = 1:length(fileNames)%%for each network 1:5
    
    fd = fopen(fullfile(path,fileNames{i}),'w');
    
    for k = 1:length(jointIndex) %%go through all the joints
      
      if(jointIndex(k,1) == i) %%if joint is associated to net i
        
        fprintf(fd,'Card%d\t',savedData(k,cardNum)); %%write card name
        
        for j = 2:numParams %%write the rest
          if( (j==ind0) | (j==ind1) | (j==ind2) ) %%must be written 0xstyle
            fprintf(fd,'%#x\t',savedData(k,j));
          else
            fprintf(fd,'%d\t',savedData(k,j));
          end
        end
        fprintf(fd,'\n');
      end
    end
    
  end