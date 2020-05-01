#ifndef MEDIAN_FILTER_H
#define MEDIAN_FILTER_H

//#define  FILTER_TIME

void bubbleSort(double *data[], int arraySize);
void swap(double *a, double *b);
void printArray(double data[], int arraySize);
double medianFilter(double rawDataArray[], int arraySize);
void shiftBuffer(double data[], int arraySize);

/*
 *    bubbleSort(data[], arraySize)
 *    Parameters: Array of data to sort
 *                Size of array
 *    Returns:    None
 *      Takes in an array of data and sorts numerically
 */
void bubbleSort(double data[], int arraySize){
  for(int i = 0; i < arraySize - 1; i++){
    for(int j = i + 1; j < arraySize; j++){
      if(data[i] > data[j]){
        swap(&data[j], &data[i]);
      }
    }
  }
}

/*
 *    swap(a, b)
 *    Parameters: pointers to a and b
 *    Returns:    None
 *      Swaps values of a and b
 */
void swap(double *a, double *b){
  double temp = *a;
  *a = *b;
  *b = temp;
}

/*
 *    printArray(data[], arraySize)
 *    Parameters: Array and size of array
 *    Returns:    None
 *      Prints a given array across one line
 */
void printArray(double data[], int arraySize){
  for(int i = 0; i < arraySize; i++){
    Serial.print(data[i], 2);Serial.print("\t");
    delay(10);
  }
  Serial.println("\n");
}

/*
 *    medianFilter(rawDaraArray[], arraySize)
 *    Parameters: Array of raw data, size of array
 *    Returns:    Median value of array
 *      Takes in an array, sorts numerically and outputs the
 *      median value.
 */
double medianFilter(double rawDataArray[], int arraySize){
  #ifdef FILTER_TIME
    unsigned long startTime = micros();
  #endif
  // temporary filter for median filter
  double tempArray[arraySize];
  // copy raw data into temporary array
  for(int i = 0; i < arraySize; i++){
    tempArray[i] = rawDataArray[i];
  }
  // sort array numerically
  bubbleSort(tempArray, arraySize);
  #ifdef  FILTER_TIME
    unsigned long filterTime = micros() - startTime;
    Serial.print("Filter time: ");Serial.print(filterTime);Serial.println(" [us]");
  #endif
  // check if length of arrayy is odd
  if(arraySize % 2 == 1){
    // median element at lenght/2
    return tempArray[arraySize / 2];
  }
  // median element average of two center elements
  return (tempArray[arraySize / 2] + tempArray[(arraySize / 2) - 1]) / 2.0;
}

/*
 *    shiftBuffer(data[], arraySize)
 *    Parameters: Array and size of array
 *    Returns:  None
 *      Shifts all values in array down one index
 */
void shiftBuffer(double data[], int arraySize){
  for(int i = 0; i < arraySize - 1; i++){
    data[i] = data[i + 1];
  }
}

#endif
