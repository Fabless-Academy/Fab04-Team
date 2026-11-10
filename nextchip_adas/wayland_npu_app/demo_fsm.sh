#!/bin/sh
# 모델 전환 시연: 단계마다 10초 (전환 판단 3초 + 1초 평균 갱신 + 화면 확인)
rm -f /tmp/fake_temp
echo "== 1. NORMAL (sensor)";              sleep 10
echo "== 2. 85C  -> LIGHT";                echo 85000 > /tmp/fake_temp; sleep 10
echo "== 3. 95C  -> SKIP";                 echo 95000 > /tmp/fake_temp; sleep 10
echo "== 4. 80C  -> LIGHT (hysteresis)";   echo 80000 > /tmp/fake_temp; sleep 10
echo "== 5. sensor -> NORMAL";             rm -f /tmp/fake_temp;        sleep 10
echo "== done"