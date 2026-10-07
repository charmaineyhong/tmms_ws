# tmms_c2 — C2 pin storage

Saves the C2 tab's mission graphs (pins on a map) so they survive a refresh or restart.

| File | What it is |
|---|---|
| `c2_backend.js` | Small server that saves/loads graphs. Loopback only, port 3002 |
| `tmms_c2-compose.yaml` | Starts MongoDB (`mongo:7`, loopback, port 27017) + `c2_backend.js` |
| `tmms_c2_supervisor.conf` | Starts the compose file on boot. Its own program, not in `quadruped` |
| `../../devops/tmms_c2.Dockerfile` | Builds `tmms_c2_image` |

The browser never talks to it directly: `ui_backend.js` forwards `/api/c2/*` to it.
Pins are stored in metres in the `map` frame. A deleted pin is never erased: it moves to the
graph's `deletedPins` with a `deletedAt` time.

API (through the proxy): `GET /api/c2/graphs` lists graphs; `PUT /api/c2/graphs/:id` with
`{name, map, pins}` saves one.

---

## Deploy to the robot, with a backup

Run everything from one terminal on the laptop, from the repo root (`cd ~/tmms/tmms_ws`).
Robot password: `Unitree0408`.

### 1. Build (laptop only)

The robot is arm64. The Dockerfile only copies files, so the laptop builds arm64 without emulation.

```bash
cd ~/tmms/tmms_ws
(cd app/tmms_c2 && npm ci --omit=dev)
docker buildx build --builder default --platform linux/arm64 \
  -f devops/tmms_c2.Dockerfile -t tmms_c2_image:0.1.0 --load app/tmms_c2
docker pull --platform linux/arm64 mongo:7
docker save --platform linux/arm64 tmms_c2_image:0.1.0 -o /tmp/tmms_c2_image_0.1.0.tar
docker save --platform linux/arm64 mongo:7 -o /tmp/mongo_7_arm64.tar
(cd app/tmms_ui && npm run build)
```

### 2. Back up what is on the robot now

```bash
R=unitree@192.168.123.165
mkdir ~/tmms/robot_backup          # fails if it already exists: never overwrite an old backup

# the UI files that the deploy replaces
rsync -a $R:/home/unitree/.htxgrrt/bin/tmms_ui/dist $R:/home/unitree/.htxgrrt/bin/tmms_ui/ui_backend.js ~/tmms/robot_backup/
ls ~/tmms/robot_backup             # must show: dist  ui_backend.js

# a record of the robot to compare against at the end
ssh $R 'docker images --format "{{.Repository}}:{{.Tag}} {{.ID}}" | sort; docker ps -a --format "{{.Names}} {{.Image}}" | sort;
  docker volume ls -q | sort; docker network ls --format "{{.Name}}" | sort; ls /etc/supervisor/conf.d; ls ~/.htxgrrt ~/.htxgrrt/bin;
  cd ~/.htxgrrt/bin/tmms_ui && find dist ui_backend.js -type f -exec md5sum {} + | sort -k2' > ~/tmms/robot_backup/before.txt

ssh $R 'docker images mongo'       # note whether mongo:7 is ALREADY on the robot
```

### 3. Deploy

```bash
# images (skip the mongo line if step 2 showed mongo:7 already there)
scp /tmp/tmms_c2_image_0.1.0.tar /tmp/mongo_7_arm64.tar $R:/tmp/
ssh $R 'docker load -i /tmp/tmms_c2_image_0.1.0.tar && rm /tmp/tmms_c2_image_0.1.0.tar'
ssh $R 'docker load -i /tmp/mongo_7_arm64.tar && rm /tmp/mongo_7_arm64.tar'

# C2 files and the MongoDB data folder
ssh $R 'mkdir -p /home/unitree/.htxgrrt/bin/tmms_c2 /home/unitree/.htxgrrt/c2/mongo'
scp app/tmms_c2/tmms_c2-compose.yaml app/tmms_c2/c2_backend.js app/tmms_c2/tmms_c2_supervisor.conf $R:/home/unitree/.htxgrrt/bin/tmms_c2/

# start it with supervisor (adds only tmms_c2; nothing else restarts)
ssh -t $R 'sudo ln -s /home/unitree/.htxgrrt/bin/tmms_c2/tmms_c2_supervisor.conf /etc/supervisor/conf.d/tmms_c2_supervisor.conf && sudo supervisorctl reread && sudo supervisorctl update'
sleep 10; ssh $R 'curl -s 127.0.0.1:3002/health'       # must print {"ok":true}

# the UI with the C2 tab
rsync -avz --delete app/tmms_ui/dist app/tmms_ui/ui_backend.js $R:/home/unitree/.htxgrrt/bin/tmms_ui/
ssh -t $R 'sudo supervisorctl restart quadruped:tmms_ui'
```

If `/health` doesn't print ok: `ssh $R 'tail -20 /home/unitree/.htxgrrt/logs/tmms_c2.log'`.

Open https://192.168.123.165:3001, press Ctrl+Shift+R, click 📍 C2.

### 4. Revert to the backup

```bash
R=unitree@192.168.123.165

# the original UI
rsync -avz --delete ~/tmms/robot_backup/dist ~/tmms/robot_backup/ui_backend.js $R:/home/unitree/.htxgrrt/bin/tmms_ui/
ssh -t $R 'sudo supervisorctl restart quadruped:tmms_ui'

# stop and remove C2 from supervisor
ssh -t $R 'sudo rm /etc/supervisor/conf.d/tmms_c2_supervisor.conf && sudo supervisorctl reread && sudo supervisorctl update'

# remove its containers, image, data, files and log
ssh $R 'docker compose -f /home/unitree/.htxgrrt/bin/tmms_c2/tmms_c2-compose.yaml down && docker rmi tmms_c2_image:0.1.0'
ssh $R 'docker rmi mongo:7'        # ONLY if mongo:7 was NOT on the robot before (step 2)
ssh -t $R 'sudo rm -rf /home/unitree/.htxgrrt/c2 /home/unitree/.htxgrrt/bin/tmms_c2 /home/unitree/.htxgrrt/logs/tmms_c2.log'

# compare with the record from step 2
ssh $R 'docker images --format "{{.Repository}}:{{.Tag}} {{.ID}}" | sort; docker ps -a --format "{{.Names}} {{.Image}}" | sort;
  docker volume ls -q | sort; docker network ls --format "{{.Name}}" | sort; ls /etc/supervisor/conf.d; ls ~/.htxgrrt ~/.htxgrrt/bin;
  cd ~/.htxgrrt/bin/tmms_ui && find dist ui_backend.js -type f -exec md5sum {} + | sort -k2' > ~/tmms/robot_backup/after.txt
diff ~/tmms/robot_backup/before.txt ~/tmms/robot_backup/after.txt && echo IDENTICAL
```

It must print `IDENTICAL`. If it doesn't, stop and read the lines `diff` printed (`<` before,
`>` after). Keep `~/tmms/robot_backup` until it says IDENTICAL; after that you can delete it.
