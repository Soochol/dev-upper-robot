set pagination off
set confirm off

target extended-remote localhost:61234

echo \n=== 초기 상태 (Reset_Handler에 자동 정지됨) ===\n
info registers pc sp

echo \n=== main 진입까지 실행 ===\n
break main
continue
info registers pc sp
bt
list

echo \n=== FreeRTOS vTaskStartScheduler까지 실행 ===\n
break vTaskStartScheduler
continue
info registers pc sp
bt

echo \n=== 스케줄러 시작 직전 FreeRTOS 상태 ===\n
print uxCurrentNumberOfTasks
print/x pxCurrentTCB

echo \n=== 스케줄러 실행 후 첫 태스크 진입 ===\n
step
info registers pc sp
print/x pxCurrentTCB

echo \n=== 현재 태스크 TCB 내용 (pcTaskName) ===\n
print (char*)((TCB_t*)pxCurrentTCB)->pcTaskName

echo \n=== RAM 덤프 (SRAM 시작) ===\n
x/16xw 0x20000000

detach
quit
