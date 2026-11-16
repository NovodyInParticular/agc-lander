// OpenGL ES builds only: rlgl calls the extension name of a function that is
// core in GLES 3.0, and the GLES libraries export only the core name.
#ifdef AGC_GLES
extern void glDrawBuffers(int n, const unsigned int *bufs);
void glDrawBuffersEXT(int n, const unsigned int *bufs) { glDrawBuffers(n, bufs); }
#endif
