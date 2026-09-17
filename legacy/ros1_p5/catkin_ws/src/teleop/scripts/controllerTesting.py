#VID_045E&PID_028E&IG_01#8&19f1ccb5&0&0000#{4d1e55b2-f16f-11cf-88cb-001111000030}',
# 'vendor_id': 1118,
# 'product_id': 654,
# 'serial_number': '1187A89',
# 'release_number': 272,
# 'manufacturer_string': 'Microsoft',
# 'product_string': 'Controller (XBOX 360 For Windows)',
# 'usage_page': 1,
# 'usage': 5,
# 'interface_number': 0,
# 'bus_type': <BusType.USB: 1>}]
import hid
import time

vendorID = 1118
productID = 654
controller = hid.Device(vendorID, productID)

while(True):
    reports = controller.read(64)
    print(reports[1], reports[3]) #left x, left y, full left is 0, right is 255, fully up is 0, down is 255
    #print(reports[5], reports[7]) #right x, right y
    time.sleep(0.1)