#ifndef STOPWATCH_HPP_
#define STOPWATCH_HPP_

#include <sys/time.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>

class Stopwatch{
    struct timeval start, end;

public:
    uint64_t u_seconds, seconds;

public:
    Stopwatch(){};

    void Start(){
        gettimeofday(&start ,NULL);
    }

    void Check(const char* event){
        gettimeofday(&end ,NULL);
        seconds = end.tv_sec - start.tv_sec;
        u_seconds = end.tv_usec - start.tv_usec;
        printf("---Stopwatch---\nevent: %s \tsec:  %ld   usec: %ld\n", event ,seconds, u_seconds);
    }

    void Check_n_Reset(const char* event){
        Check(event);
        Start();
    }
};

#endif
