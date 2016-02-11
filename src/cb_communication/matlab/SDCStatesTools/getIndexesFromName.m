function index = getIndexesFromName(name,nameList)
  
  for i = 1:length(nameList)
    if(strcmp(nameList(i),name))
      index = i;
      return;
    end
  end
  
  index = -1;