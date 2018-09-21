GOARM=6 GOARCH=arm GOOS=linux go build -o main raspi_bme680.go
scp main pi@192.168.1.111:
ssh -t pi@192.168.1.111 "./main"