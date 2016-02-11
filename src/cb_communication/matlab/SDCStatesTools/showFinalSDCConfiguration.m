allParamsMapFileName = 'allSDCParamsMap.conf';
writeParamsMapFileName = 'writeableSDCParamsMap.conf';
jointMapFileName = 'jointMap.conf';

path = sprintf('%s/cb_communication', getenv('LAB_ROOT'));
nameRoot = 'finalSDCState';
%nameRoot = 'currentSDCState';
rawDataTable = generateVisual(allParamsMapFileName,writeParamsMapFileName,jointMapFileName,path,nameRoot);
