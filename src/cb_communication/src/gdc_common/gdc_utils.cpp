/*!=============================================================================
  ==============================================================================

  \file    gdc_utils.cpp

  \author  righetti
  \date    May 16, 2012

  ==============================================================================
  \remarks useful functions for gdc stuff
  
  
  ============================================================================*/


#include <utility.h>


namespace hermes_communication_tools
{

bool read_config_int_array(char *fname, char *keyword, int n_values, int *ivalues)
{
  int rc;
  FILE  *in;

  in = fopen_strip(fname);
  if (in == NULL) {
    printf("ERROR: Cannot open file >%s<!\n",fname);
    return false;
  }

  // find keyword
  if (!find_keyword(in, keyword)) {
    fclose(in);
    return false;
  } else {
    for (int i=0; i<n_values; ++i) {
      rc=fscanf(in,"%d",&(ivalues[i]));
      if (rc != 1)
      {
        fclose(in);
	printf("couldn't find enough parameters %d rc %d\n", i, rc);
        return false;
      }
    }
  }

  fclose(in);

  return true;
}

}
