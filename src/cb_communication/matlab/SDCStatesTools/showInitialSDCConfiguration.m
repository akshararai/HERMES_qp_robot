allParamsMapFileName = 'allSDCParamsMap.conf';
writeParamsMapFileName = 'writeableSDCParamsMap.conf';
jointMapFileName = 'jointMap.conf';

path = sprintf('%s/cb_communication/config', getenv('LAB_ROOT'));
nameRoot = 'initialSDCState';

rawDataTable = generateVisual(writeParamsMapFileName,writeParamsMapFileName,jointMapFileName,path,nameRoot);
