
import itchat
import requests

global AutoReplay
AutoReplay=True
def GetResponse(msg):
    apiUrl = 'http://www.tuling123.com/openapi/api'
    data = {
        'key'    : '4d84bed3cb6ecc150fe9dc5493f321ea', # 如果这个Tuling Key不能用，那就换一个
        'info'   : msg, # 这是我们发出去的消息
        'userid' : '2385909406@qq.com', # 这里你想改什么都可以
    }
    try:
        # 我们通过如下命令发送一个post请求
        r = requests.post(apiUrl, data=data).json()
    
        # 让我们打印一下返回的值，看一下我们拿到了什么
        return r.get('text')
    except:
        return 'error'
@itchat.msg_register(itchat.content.TEXT)
def print_content(msg):
    global AutoReplay
    #print msg
    user=msg['User']
    name=''
    try:
        name=user['RemarkName']
        print name
        print user['NickName']
    except:
        print 'error'
    message=msg['Text']
    if 'sr' in message:
        AutoReplay=False
        itchat.send(u'自动回复关闭', 'filehelper')
    if 'ar' in message:
        AutoReplay=True
        itchat.send(u'自动回复开启', 'filehelper')
        
    #print user['NickName']
    #itchat.send(u'我的主人正忙，请稍后联系', 'filehelper')
    #defaultReply = 'I received: ' + msg['Text']
    #itchat.send(msg['Text'], 'filehelper')
    res=GetResponse(msg['Text'])
    #itchat.send(res, 'filehelper')
    if('MM' in name):
        if AutoReplay:
            #res=GetResponse(msg['Text'])
            return res
    #itchat.send(res, 'filehelper')
    #return msg['Text']
    return res

itchat.auto_login(hotReload=True)
itchat.run()
