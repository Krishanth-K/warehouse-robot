import qrcode

qr = qrcode.QRCode(box_size=8, border=4)
qr.add_data("ID=83")
qr.make()
img = qr.make_image()
img = img.resize((512, 512))
img.save("human_03_qr.png")
