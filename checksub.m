sub = rossubscriber('/test_com',@test_com_call);
pause(1)
global x;
for i=1:300
   x=x+1
   pause(1)
end