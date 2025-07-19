#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/skbuff.h>
#include <linux/dma-mapping.h>

#define MAX_PACKET_SIZE 				1500  					
#define NETWORK_DEVICE_REG_BASE     	0x40000000  
#define NETWORK_DEVICE_MAC_HIGH_REG 	0x10        
#define NETWORK_DEVICE_MAC_LOW_REG  	0x14        
#define NETWORK_DEVICE_RX_MODE_REG    	0x20      
#define RX_MODE_PROMISCUOUS           	0x1       
#define RX_MODE_ALL_MULTICAST         	0x2       
#define NETWORK_DEVICE_VLAN_TABLE_BASE  0x100   
#define NETWORK_DEVICE_VLAN_TABLE_SIZE  4096    
#define NETWORK_DEVICE_RX_PACKETS_REG   (NETWORK_DEVICE_REG_BASE + 0x010)
#define NETWORK_DEVICE_TX_PACKETS_REG   (NETWORK_DEVICE_REG_BASE + 0x014)
#define NETWORK_DEVICE_RX_BYTES_REG     (NETWORK_DEVICE_REG_BASE + 0x018)
#define NETWORK_DEVICE_TX_BYTES_REG     (NETWORK_DEVICE_REG_BASE + 0x01C)
#define DMA_BUFFER_ADDRESS   			0x100000  			
#define DMA_CONTROL_REGISTER 			0x200000  			
#define PACKET_SIZE 					64  						
#define IRQ_LINE 						10 							
#define INTERRUPT_STATUS_REGISTER 		0x10 			
#define RX_INTERRUPT 					0x01 						
#define TX_INTERRUPT 					0x02 						
#define ERROR_INTERRUPT 				0x04 					
#define RESET_COMMAND					0xFF 						
#define RESET_REGISTER_ADDRESS 			0x20 			
#define NETWORK_DEVICE_MC_HIGH_REG   	0xYOUR_MC_HIGH_REG_OFFSET
#define NETWORK_DEVICE_MC_LOW_REG   	0xYOUR_MC_LOW_REG_OFFSET
#define NETWORK_DEVICE_CMD_REG       	0xYOUR_CMD_REG_OFFSET
#define YOUR_MC_HIGH_REG_OFFSET 		0x1234 			
#define YOUR_MC_LOW_REG_OFFSET   		0x5678  		
#define YOUR_CMD_REG_OFFSET      		0x9ABC  		
#define MIN_MTU 						68
#define MAX_MTU 						1500


static struct net_device *network_ndev;  
static struct napi_struct network_napi;  

static int network_set_mac_address(struct net_device *dev, void *p) { 

    struct sockaddr *addr = p;
    u32 mac_high, mac_low;

    if (!is_valid_ether_addr(addr->sa_data))
        return -EADDRNOTAVAIL;

    memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);

    mac_high = *(u32 *)(addr->sa_data);
    mac_low = *(u16 *)(addr->sa_data + 4);
    
    iowrite32(mac_high, (void __iomem *)(NETWORK_DEVICE_REG_BASE + NETWORK_DEVICE_MAC_HIGH_REG));
    iowrite32(mac_low, (void __iomem *)(NETWORK_DEVICE_REG_BASE + NETWORK_DEVICE_MAC_LOW_REG));

    return 0;
}


void write_multicast_address_to_hardware(const u8 *addr) {	

    u32 low_part, high_part;

    high_part = ((u32)addr[0] << 8) | addr[1];
    low_part = ((u32)addr[2] << 24) | ((u32)addr[3] << 16) | ((u32)addr[4] << 8) | addr[5];

    iowrite32(high_part, (void __iomem *)(NETWORK_DEVICE_REG_BASE + NETWORK_DEVICE_MC_HIGH_REG));
    iowrite32(low_part, (void __iomem *)(NETWORK_DEVICE_REG_BASE + NETWORK_DEVICE_MC_LOW_REG));
    iowrite32(COMMAND_WRITE_MULTICAST, (void __iomem *)(NETWORK_DEVICE_REG_BASE + NETWORK_DEVICE_CMD_REG));
}

static void network_set_multicast_list(struct net_device *dev) {
    u32 rx_mode = 0;

    if (dev->flags & IFF_PROMISC) {	
        rx_mode |= RX_MODE_PROMISCUOUS;  
    } 
    else if (dev->flags & IFF_ALLMULTI) {
        rx_mode |= RX_MODE_ALL_MULTICAST;
    } 
    else {
        struct netdev_hw_addr *ha;
        netdev_for_each_mc_addr(ha, dev) {
            write_multicast_address_to_hardware(ha->addr);
        }
    }
    iowrite32(rx_mode, (void __iomem *)(NETWORK_DEVICE_REG_BASE + NETWORK_DEVICE_RX_MODE_REG));
}


static int network_vlan_rx_add_vid(struct net_device *dev, __be16 proto, u16 vid) { 
    if (vid < NETWORK_DEVICE_VLAN_TABLE_SIZE) {
    	
        iowrite16(vid, (void __iomem *)(NETWORK_DEVICE_REG_BASE + NETWORK_DEVICE_VLAN_TABLE_BASE + vid * sizeof(u16)));
        return 0;
    }
    return -EINVAL;
}

static int network_vlan_rx_kill_vid(struct net_device *dev, __be16 proto, u16 vid) {
    if (vid < NETWORK_DEVICE_VLAN_TABLE_SIZE) {
    	
        iowrite16(0xFFFF, (void __iomem *)(NETWORK_DEVICE_REG_BASE + NETWORK_DEVICE_VLAN_TABLE_BASE + vid * sizeof(u16)));
        return 0;
    }
    return -EINVAL;
}

struct net_device_stats *network_get_stats(struct net_device *dev) {
    static struct net_device_stats stats;
    stats.rx_packets = ioread32((void __iomem *)NETWORK_DEVICE_RX_PACKETS_REG);
    stats.tx_packets = ioread32((void __iomem *)NETWORK_DEVICE_TX_PACKETS_REG);
    stats.rx_bytes   = ioread32((void __iomem *)NETWORK_DEVICE_RX_BYTES_REG);
    stats.tx_bytes   = ioread32((void __iomem *)NETWORK_DEVICE_TX_BYTES_REG);

    return &stats;
}

static int network_poll(struct napi_struct *napi, int budget) {

    struct sk_buff *skb;
    int processed = 0;

    while (processed < budget) {
        skb = hardware_receive_packet();

        if (!skb) {
            break;
        }
        netif_receive_skb(skb);
        processed++;
    }

    if (processed < budget) {
        napi_complete_done(napi, processed);
    }

    return processed;
}


static void network_reset_hardware(void) {
    iowrite32(RESET_COMMAND, (void __iomem *)(NETWORK_DEVICE_REG_BASE + RESET_REGISTER_ADDRESS));
}


static struct sk_buff *hardware_receive_packet(void) {
    dma_addr_t dma_addr;
    void *dma_buffer;
    struct sk_buff *skb;
    size_t received_len;  

    dma_buffer = dma_alloc_coherent(NULL, MAX_PACKET_SIZE, &dma_addr, GFP_KERNEL);
    if (!dma_buffer) {
        printk(KERN_ERR "Could not allocate DMA memory!\n");
        return NULL;
    }

    printk(KERN_INFO "Data retrieved from %lx\n", (unsigned long)dma_addr);
    
    received_len = 100;  

    skb = dev_alloc_skb(received_len + 2);
    
    if (!skb) {
        printk(KERN_ERR "Failed to allocate memory for sk_buff!\n");
        dma_free_coherent(NULL, MAX_PACKET_SIZE, dma_buffer, dma_addr);
        return NULL;
    }

    skb_reserve(skb, 2);  
    skb_copy_to_linear_data(skb, dma_buffer, received_len);
    skb_put(skb, received_len);

    dma_free_coherent(NULL, MAX_PACKET_SIZE, dma_buffer, dma_addr);

    return skb;
}

static void hardware_transmit_packet(struct sk_buff *skb) {
    dma_addr_t dma_addr;
    void *dma_buffer;
    size_t packet_len = skb->len;

    dma_buffer = dma_alloc_coherent(NULL, packet_len, &dma_addr, GFP_KERNEL);
    if (!dma_buffer) {
        printk(KERN_ERR "Could not allocate DMA memory!\n");
        return;
    }

    skb_copy_from_linear_data(skb, dma_buffer, packet_len);

    void __iomem *dma_buffer_mapped = ioremap(DMA_BUFFER_ADDRESS, skb->len);
    
    if (dma_buffer_mapped) {
        memcpy_toio(dma_buffer_mapped, dma_buffer, packet_len);
        iounmap(dma_buffer_mapped);
    } else {
        printk(KERN_ERR "DMA buffer mapping error!\n");
        dma_free_coherent(NULL, packet_len, dma_buffer, dma_addr);
        return;
    }

    void __iomem *control_reg = ioremap(DMA_CONTROL_REGISTER, sizeof(u32));
    if (control_reg) {
        iowrite32(0x1, control_reg);  
        iounmap(control_reg);
    } else {
        printk(KERN_ERR "Control register mapping error!\n");
    }

    dma_free_coherent(NULL, packet_len, dma_buffer, dma_addr);
}



static irqreturn_t network_interrupt(int irq, void *dev_id) { 
    u32 interrupt_status;
    struct net_device *dev = (struct net_device *) dev_id;
    struct sk_buff *skb;
    int bytes_read;

    
    if (napi_schedule_prep(&network_napi)) {
        __napi_schedule(&network_napi);
    }

    interrupt_status = ioread32((void __iomem *)INTERRUPT_STATUS_REGISTER);
    
    if (interrupt_status & RX_INTERRUPT) {
        skb = dev_alloc_skb(MAX_FRAME_SIZE);
        if (!skb) {
            return IRQ_NONE;
        }

        bytes_read = read_data_from_hardware(skb->data);

        if (bytes_read > 0) {
            skb_put(skb, bytes_read);
            netif_rx(skb);
        } else {
            dev_kfree_skb(skb);
        }
    }

    if (interrupt_status & TX_INTERRUPT) {
        netif_wake_queue(dev);
    }

    if (interrupt_status & ERROR_INTERRUPT) {
        dev->stats.tx_errors++;
        dev->stats.rx_errors++;
    }    

    return IRQ_HANDLED;
}



static netdev_tx_t network_start_xmit(struct sk_buff *skb, struct net_device *dev) {

    hardware_transmit_packet(skb);    
    printk(KERN_INFO "NAPI Network Driver: Transmit function called\n");
    dev_kfree_skb(skb);

    return NETDEV_TX_OK;
}


static int my_network_change_mtu(struct net_device *dev, int new_mtu) {
    
    if (new_mtu < MIN_MTU || new_mtu > MAX_MTU) {
        printk(KERN_WARNING "my_network: MTU value out of range. It should be between %d and %d.\n", MIN_MTU, MAX_MTU);
        return -EINVAL; 
    }

    dev->mtu = new_mtu;

    printk(KERN_INFO "my_network: MTU updated to %d\n", new_mtu);


    return 0; 
}


static int my_network_set_config(struct net_device *dev, struct ifmap *map) {

    if (map->base_addr != (unsigned long)(-1) && map->base_addr != dev->base_addr) {
        dev->base_addr = map->base_addr;
        printk(KERN_INFO "my_network: base_addr updated to %lx\n", dev->base_addr);
    }

    if (map->irq != (unsigned char)(-1) && map->irq != dev->irq) {
        dev->irq = map->irq;
        printk(KERN_INFO "my_network: irq updated to %d\n", dev->irq);
    }

    if (map->dma != (unsigned char)(-1) && map->dma != dev->dma) {
        dev->dma = map->dma;
        printk(KERN_INFO "my_network: dma updated to %d\n", dev->dma);
    }

    return 0; 
}


static int network_open(struct net_device *dev) {
    
    netif_start_queue(dev);

    for (int i = 0; i < DMA_RING_SIZE; i++) {
        
        dma_ring[i].buffer = dma_alloc_coherent(dev, BUFFER_SIZE, &dma_ring[i].dma_handle, GFP_KERNEL);
        
        if (!dma_ring[i].buffer) {
            printk(KERN_ERR "Failed to allocate DMA buffer\n");
            
            for (int j = 0; j < i; j++) {
                dma_free_coherent(dev, BUFFER_SIZE, dma_ring[j].buffer, dma_ring[j].dma_handle);
            }
            return -ENOMEM;
        }
    }

    int ret = request_irq(IRQ_LINE, network_interrupt, IRQF_SHARED, "my_device_name", dev);
    
    if (ret) {
        printk(KERN_ERR "Failed to request IRQ\n");
        
        for (int i = 0; i < DMA_RING_SIZE; i++) {
            if (dma_ring[i].buffer) {
                dma_free_coherent(dev, BUFFER_SIZE, dma_ring[i].buffer, dma_ring[i].dma_handle);
            }
        }
        return ret;
    }

    napi_enable(&network_napi);

    return 0;
}

static int network_stop(struct net_device *dev) {

    netif_stop_queue(dev);
    free_irq(IRQ_LINE, dev);
    napi_disable(&network_napi);

    for (int i = 0; i < DMA_RING_SIZE; i++) {

        if (dma_ring[i].buffer)
            dma_free_coherent(dev, BUFFER_SIZE, dma_ring[i].buffer, dma_ring[i].dma_handle);
    }

    return 0;
}

static const struct net_device_ops network_netdev_ops = {
    .ndo_open = network_open,
    .ndo_stop = network_stop,
    .ndo_start_xmit = network_start_xmit,
    .ndo_set_mac_address = network_set_mac_address,
    .ndo_set_rx_mode = network_set_multicast_list,
    .ndo_vlan_rx_add_vid = network_vlan_rx_add_vid,
    .ndo_vlan_rx_kill_vid = network_vlan_rx_kill_vid,
    .ndo_get_stats = network_get_stats,    
    .ndo_set_config = my_network_set_config,     
    .ndo_change_mtu = my_network_change_mtu,
};

static int __init network_init(void) {
    int ret;

    network_ndev = alloc_etherdev(sizeof(struct net_device));
    if (!network_ndev) {
        printk(KERN_ERR "NAPI Network Driver: Failed to allocate network device\n");
        return -ENOMEM;  
    }

    network_ndev->netdev_ops = &network_netdev_ops;
    network_ndev->flags |= IFF_PROMISC;
    netif_napi_add(network_ndev, &network_napi, network_poll, 64);  

    ret = register_netdev(network_ndev);
    if (ret) {
        printk(KERN_ERR "NAPI Network Driver: Failed to register network device\n");
        free_netdev(network_ndev);  
        return ret;
    }
    
    ret = request_irq(IRQ_LINE, network_interrupt, IRQF_SHARED, "network_irq", network_ndev);
    if (ret) {
        printk(KERN_ERR "NAPI Network Driver: Failed to request IRQ\n");
        unregister_netdev(network_ndev);  
        free_netdev(network_ndev);  
        return ret;
    }

    printk(KERN_INFO "NAPI Network Driver: Initialized\n");

    return 0;  
}

static void network_exit(void)
{
    free_irq(IRQ_LINE, network_ndev);
    napi_disable(&network_napi);
    napi_del(&network_napi);

    if (network_ndev) {
        unregister_netdev(network_ndev);        
        free_netdev(network_ndev);
    }

    printk(KERN_INFO "NAPI Network Driver: Module unloaded\n");
}

module_init(network_init);
module_exit(network_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NAPI Network Driver");
MODULE_AUTHOR("Resul");