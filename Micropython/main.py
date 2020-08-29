def take_sample():

  sensor.take_measurements_with_bulb()

  sample = sensor.get_calibrated_A(), sensor.get_calibrated_B(), sensor.get_calibrated_C(), sensor.get_calibrated_D(), sensor.get_calibrated_E(), sensor.get_calibrated_F(), sensor.get_calibrated_G(), sensor.get_calibrated_H(), sensor.get_calibrated_R(), sensor.get_calibrated_I(), sensor.get_calibrated_S(), sensor.get_calibrated_J(), sensor.get_calibrated_T(), sensor.get_calibrated_U(), sensor.get_calibrated_V(), sensor.get_calibrated_W(), sensor.get_calibrated_K(), sensor.get_calibrated_L()

  return sample

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('', 80))
s.listen(5)


while True:
  try:
    if gc.mem_free() < 102000:
      gc.collect()
    conn, addr = s.accept()
    conn.settimeout(3.0)
    print('Got a connection from %s' % str(addr))
    request = conn.recv(1024)
    conn.settimeout(None)
    request = str(request)
    print('Content = %s' % request)
    sample = str(take_sample())
    conn.send('HTTP/1.1 200 OK\n')
    conn.send('Content-Type: text/html\n')
    conn.send('Connection: close\n\n')
    conn.sendall(sample)
    conn.close()
  except OSError as e:
    conn.close()
    print('Connection closed')
