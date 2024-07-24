/**
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All right reserved.
**/

/**
Header analysis by bon, June 17, 2024
**/

#ifndef _UNITREE_LEGGED_LOOP_H_
#define _UNITREE_LEGGED_LOOP_H_

#include <stdio.h>
#include <stdlib.>
#include <string.h>
#include <thread>
#include <pthread.h>
#include <vector>
#include <boost/shared_ptr_hpp>
#include <boost/function.hpp>
//#include <pybindll/pybindll.h>

namespace UNITREE_LEGGED_SDK
{
  constexpr int THREAD_PRIORITY = 95; //real-time priority, 스레드의 우선 순위를 설정하는 상수로 실시간 우선 순위로 설정되어 있으며 값은 95

  typedef boost::function<void()> Callback; //Callback 타입은 반환 값이 없고 매개 변수가 없는 함수 객체를 나타냄, boost::function을 사용하여 콜백 함수를 정의

  class Loop
  {
    public: 
      Loop(std::string name, float period, int bindCPU = -1):_name(name), _period(period), _bindCPU(bindCPU) {} //생성자로, Loop 객체 초기화, name: 루프의 이름, perod: 루프의 주기, bindCPU: CPU 바인딩(기본값 -1)
      ~Loop(); //소멸자

      void start(); //루프 시작, _isrunning 플래그를 설정하고, entryFunc를 새로운 스레드에서 실행
      void shutdown(); //루프 종료, _isrunning 플래그를 해제하고, 스레드가 종료될 때까지 기다림
      virtual void functionCB() = 0; //순수 가상 함수

    private:
      void entryFunc(); //실제 루프 실행을 담당하는 함수, 스레드에서 실행되며 _isrunning 플래그가 true인 동안 _period 주기로 functionCB 호출

      std::string _name; //루프의 이름을 나타내는 문자열
      float _period; //루프의 주기를 나타내는 부동 소수점
      int _bindCPU; //특정 CPU에 바인딩 하기 위한 정수, -1이면 특정 CPU에 바인딩 하지 않음
      bool _bind_cpu_flag = false; //CPU 바인딩 여부를 나타내는 플래그로 기본값은 false
      bool _isrunning = false; //루프가 실행 중인지 여부를 나타내는 플래그로 기본값은 false
      std::thread _thread; //루프를 실행하는 스레드 객체

  /**
  class Pyloop: public Loop
  {
    public: 
      using Loop::Loop;
  
    void functionCB() override
    {
      PYBIND11_OVERRIDE_PURE(
        void,
        Loop,
        functionCB
      );
    }
  };
  **/

/*
period unit: seconde
bindCPU change the CPU affinity of this thread
*/

  class LoopFunc : public Loop //LoopFunc 클래스는 Loop 클래스를 상속 받음
  {
    public:
      LoopFunc(std::string name, float period, const Callback& _cb) //생성자 1, 상위 클래스 Loop의 생성자를 호출하여 name과 period를 초기화
        : Loop(name, period), _fp(_cb){} //콜백 함수 객체 _fp를 _cb로 초기화
      LoopFunc(std::string name, float period, int bindCPU, const Callback& _cb) //생성자 2, 상위 클래스 Loop의 생성자를 호출하여 name과 period 및 bindCPU를 초기화
        : Loop(name, period, bindCPU), _fp(_cb){} //콜백 함수 객체 _fp를 _cb로 초기화
      void functionCB() {(_fp)();} //순수 가상 함수 functionCB를 구현

    private:
      boost::function<void()> _fp; //콜백 함수를 저장하는 변수로 boost::function<void ()> 타입으로, 반환 값이 없고 매개 변수가 없는 함수 객체를 나타냄, 생성자에게 전달된 콜백 함수 객체 _cb로 초기화
  };
}

#endif
      
