import urllib.request
import urllib.parse

url = 'http://192.168.11.190/relay/0?turn=on'
urllib.request.urlopen(url)
url = 'http://192.168.11.191/relay/0?turn=on'
urllib.request.urlopen(url)
