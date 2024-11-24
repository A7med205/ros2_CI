#### Install and Start Docker
```
sudo apt-get update
sudo apt-get install -y docker.io docker-compose
sudo service docker start
```

#### Start Jenkins and Open Webpage
```
cd ~/webpage_ws
bash start_jenkins.sh
cat /home/user/jenkins__pid__url.txt
```

#### Login to Jenkins
```
User: admin
Password: pineapple
```

#### Start Jenkins Job
- `echo "$(jenkins_address)github-webhook/"`
- Send me webhook address and github email
- Create pull request to `github.com/A7med205/ros2_CI`