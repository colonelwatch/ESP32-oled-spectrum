template <typename TYPE, int SIZE> class doubleBuffer{
    public:
        volatile TYPE *readBuffer, *writeBuffer;
        void swap(){
            volatile TYPE *temp = readBuffer;
            readBuffer = writeBuffer;
            writeBuffer = temp;
        }
        void alloc(){
            readBuffer = (TYPE*)malloc(SIZE*sizeof(TYPE));
            writeBuffer = (TYPE*)malloc(SIZE*sizeof(TYPE));
        }
};

// for now, make BUF_SIZE "significantly" bigger than WINDOW_SIZE to prevent
//   writing into the range being read
// TODO: use assertions to tightly test for the smallest reasonable buffer size
template <typename TYPE, int WINDOW_SIZE, int BUF_SIZE> class fftBuffer{
    public:
        void write(const TYPE *data, int size){
            int end_index_copy = end_index; // take a copy volatile end_index for consistency

            int i_start = end_index_copy, i_end = (end_index_copy+size)%BUF_SIZE;
            for(int i = i_start, j = 0; i != i_end; i = (i+1)%BUF_SIZE, j++) buffer[i] = data[j];

            end_index = i_end; // advance volatile end_index
        }
        void read(TYPE *data){
            int end_index_copy = end_index;
            
            int i_start = end_index_copy-WINDOW_SIZE;
            if(i_start < 0) i_start += BUF_SIZE;
            int i_end = end_index_copy;
            for(int i = i_start, j = 0; i != i_end; i = (i+1)%BUF_SIZE, j++) data[j] = buffer[i];
        }
        void alloc(){ buffer = (TYPE*)calloc(BUF_SIZE, sizeof(TYPE)); }
    private:
        volatile TYPE *buffer;
        volatile int end_index = 0;
};