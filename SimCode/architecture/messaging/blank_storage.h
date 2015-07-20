
#ifndef _BlankStorage_HH_
#define _BlankStorage_HH_

#include <vector>
#include <stdint.h>

class BlankStorage
{

 public:
   BlankStorage();
   ~BlankStorage();
   void IncreaseStorage(uint64_t NewVolume);
   void ClearStorage();
   uint64_t GetCurrentSize() {return(BufferStorageSize);}

 public:
   uint8_t* StorageBuffer;  
 private:  
   uint64_t BufferStorageSize;
};

#endif /* _BlankStorage_H_ */
