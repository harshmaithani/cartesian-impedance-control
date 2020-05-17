/***********************************************************************************************************/
float** Make2DFloatArray(int arraySizeX, int arraySizeY) 
{  
  int i;
  float** theArray;  
  theArray = (float**) malloc(arraySizeX*sizeof(float*));  

  for (i = 0; i < arraySizeX; i++)  
       theArray[i] = (float*) malloc(arraySizeY*sizeof(float));  

  return theArray;  
}
 
/***********************************************************************************************************/
float* Make1DFloatArray(int arraySizeX) 
{  
  float* theArray;  
  theArray = (float*) malloc(arraySizeX*sizeof(float));  

  return theArray;  
}
/***********************************************************************************************************/
float* zeros1DFloatArray(int arraySizeX) 
{  
  float* theArray;  
  theArray = (float*) malloc(arraySizeX*sizeof(float));  

  int i;
  for(i=0;i<arraySizeX;i++) theArray[i] = 0.0;

  return theArray;  
}
/***********************************************************************************************************/
  
  /***********************************************************************************************************/
float** zeros2DFloatArray(int arraySizeX, int arraySizeY) 
{  
  int i,j;
  float** theArray;  
  theArray = (float**) malloc(arraySizeX*sizeof(float*));  

  for (i = 0; i < arraySizeX; i++)  
       theArray[i] = (float*) malloc(arraySizeY*sizeof(float));  

    for (i = 0; i < arraySizeX; i++)
        for (j = 0;j < arraySizeY; j++)
	  theArray[i][j] = 0.0; 
  
  
  return theArray;  
}

