gz service -s /world/empty/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 3000 \
  --req 'sdf: "<?xml version=\"1.0\"?><sdf version=\"1.9\"><model name=\"notebooks\"><static>true</static><pose>0.45 0 0.075 0 0 0</pose><link name=\"link\"><collision name=\"collision\"><geometry><box><size>0.20 0.25 0.15</size></box></geometry></collision><visual name=\"visual\"><geometry><box><size>0.20 0.25 0.15</size></box></geometry><material><ambient>0.8 0.2 0.2 1</ambient><diffuse>0.8 0.2 0.2 1</diffuse></material></visual></link></model></sdf>"'
