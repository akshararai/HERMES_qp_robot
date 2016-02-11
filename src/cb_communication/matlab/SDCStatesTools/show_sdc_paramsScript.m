allParamsMapFileName = 'allSDCParamsMap.conf';
writeParamsMapFileName = 'writeableSDCParamsMap.conf';
jointMapFileName = 'jointMap.conf';

path = '~/workspace/SL_root/cb_communication/';
nameRoot = 'finalSDCState';
%nameRoot = 'currentSDCState';
rawDataTable = generateVisual(allParamsMapFileName,writeParamsMapFileName,jointMapFileName,path,nameRoot);


path = '~/workspace/SL_root/cb_communication/config';
nameRoot = 'initialSDCState';

rawDataTable2 = generateVisual(writeParamsMapFileName,writeParamsMapFileName,jointMapFileName,path,nameRoot);
