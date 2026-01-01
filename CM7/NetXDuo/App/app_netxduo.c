/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_netxduo.c
  * @brief   NetXDuo applicative file - TCP IPv4 Server (ST-style lifecycle)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "app_netxduo.h"

/* USER CODE BEGIN Includes */
#include "main.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
/* TCP server configuration */
#define TCP_SERVER_PORT               6000U
#define TCP_WINDOW_SIZE               (1460U * 2U)   /* 2*MSS */
#define TCP_RECONNECT_TIMEOUT_TICKS   (NX_IP_PERIODIC_RATE) /* ~1s in NetX ticks */

#define PRINT_IP_ADDRESS(addr)        do { \
  printf("Device IPv4 Address: %lu.%lu.%lu.%lu\r\n", \
         ((addr) >> 24) & 0xFF, \
         ((addr) >> 16) & 0xFF, \
         ((addr) >>  8) & 0xFF, \
         ((addr)      ) & 0xFF); \
} while(0)
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
TX_THREAD      NxAppThread;
NX_PACKET_POOL NxAppPool;
NX_IP          NetXDuoEthIpInstance;

/* USER CODE BEGIN PV */
static NX_TCP_SOCKET TCPSocket;
static ULONG IpAddress;
static ULONG NetMask;

extern UART_HandleTypeDef huart3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static VOID nx_app_thread_entry(ULONG thread_input);

/* USER CODE BEGIN PFP */
static UINT tcp_server_init(NX_IP *ip, NX_TCP_SOCKET *sock);
static VOID tcp_server_run(NX_IP *ip, NX_TCP_SOCKET *sock);
/* USER CODE END PFP */

UINT MX_NetXDuo_Init(VOID *memory_ptr)
{
  UINT ret = NX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;
  CHAR *pointer;

  nx_system_initialize();

  /* Packet pool */
  if (tx_byte_allocate(byte_pool, (VOID**)&pointer, NX_APP_PACKET_POOL_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  ret = nx_packet_pool_create(&NxAppPool, "NetXDuo App Pool", DEFAULT_PAYLOAD_SIZE,
                              pointer, NX_APP_PACKET_POOL_SIZE);
  if (ret != NX_SUCCESS)
  {
    printf("nx_packet_pool_create failed: 0x%02X\r\n", (unsigned int)ret);
    return NX_POOL_ERROR;
  }

  /* IP instance memory */
  if (tx_byte_allocate(byte_pool, (VOID**)&pointer, Nx_IP_INSTANCE_THREAD_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  /* IP instance */
  ret = nx_ip_create(&NetXDuoEthIpInstance, "NetX Ip instance",
                     NX_APP_DEFAULT_IP_ADDRESS, NX_APP_DEFAULT_NET_MASK,
                     &NxAppPool, nx_stm32_eth_driver,
                     pointer, Nx_IP_INSTANCE_THREAD_SIZE, NX_APP_INSTANCE_PRIORITY);
  if (ret != NX_SUCCESS)
  {
    printf("nx_ip_create failed: 0x%02X\r\n", (unsigned int)ret);
    return NX_NOT_SUCCESSFUL;
  }

  /* ARP cache */
  if (tx_byte_allocate(byte_pool, (VOID**)&pointer, DEFAULT_ARP_CACHE_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  ret = nx_arp_enable(&NetXDuoEthIpInstance, (VOID*)pointer, DEFAULT_ARP_CACHE_SIZE);
  if (ret != NX_SUCCESS)
  {
    printf("nx_arp_enable failed: 0x%02X\r\n", (unsigned int)ret);
    return NX_NOT_SUCCESSFUL;
  }

  /* ICMP (optional but useful for ping bring-up) */
  ret = nx_icmp_enable(&NetXDuoEthIpInstance);
  if (ret != NX_SUCCESS)
  {
    printf("nx_icmp_enable failed: 0x%02X\r\n", (unsigned int)ret);
    return NX_NOT_SUCCESSFUL;
  }

  /* TCP */
  ret = nx_tcp_enable(&NetXDuoEthIpInstance);
  if (ret != NX_SUCCESS)
  {
    printf("nx_tcp_enable failed: 0x%02X\r\n", (unsigned int)ret);
    return NX_NOT_SUCCESSFUL;
  }

  /* UDP (not required for pure TCP server; keep enabled only if you use DHCP etc.) */
  ret = nx_udp_enable(&NetXDuoEthIpInstance);
  if (ret != NX_SUCCESS)
  {
    printf("nx_udp_enable failed: 0x%02X\r\n", (unsigned int)ret);
    return NX_NOT_SUCCESSFUL;
  }

  /* Main thread stack */
  if (tx_byte_allocate(byte_pool, (VOID**)&pointer, NX_APP_THREAD_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  ret = tx_thread_create(&NxAppThread, "NetXDuo App thread",
                         nx_app_thread_entry, 0,
                         pointer, NX_APP_THREAD_STACK_SIZE,
                         NX_APP_THREAD_PRIORITY, NX_APP_THREAD_PRIORITY,
                         TX_NO_TIME_SLICE, TX_AUTO_START);
  if (ret != TX_SUCCESS)
  {
    printf("tx_thread_create failed: 0x%02X\r\n", (unsigned int)ret);
    return TX_THREAD_ERROR;
  }

  return NX_SUCCESS;
}

/* USER CODE BEGIN 1 */

static VOID nx_app_thread_entry(ULONG thread_input)
{
  UINT ret;
  NX_PACKET *ping_response = NX_NULL;

  (void)thread_input;

  /* Print IPv4 */
  ret = nx_ip_address_get(&NetXDuoEthIpInstance, &IpAddress, &NetMask);
  if (ret != NX_SUCCESS)
  {
    printf("nx_ip_address_get failed: 0x%02X\r\n", (unsigned int)ret);
    Error_Handler();
  }
  PRINT_IP_ADDRESS(IpAddress);

  /* Optional bring-up test: self ping */
  ret = nx_icmp_ping(&NetXDuoEthIpInstance,
                     IpAddress,
                     "PING",
                     4,
                     &ping_response,
                     NX_IP_PERIODIC_RATE);
  if (ret == NX_SUCCESS)
  {
    printf("ICMP self-ping OK\r\n");
    if (ping_response) nx_packet_release(ping_response);
  }
  else
  {
    printf("ICMP self-ping FAILED: 0x%02X\r\n", (unsigned int)ret);
    /* Keep running even if ping fails; comment Error_Handler() if desired */
  }

  /* Create socket + LISTEN ONCE (ST-style) */
  ret = tcp_server_init(&NetXDuoEthIpInstance, &TCPSocket);
  if (ret != NX_SUCCESS)
  {
    printf("TCP server init failed: 0x%02X\r\n", (unsigned int)ret);
    Error_Handler();
  }

  /* Accept/recv loop forever with relisten after disconnect */
  tcp_server_run(&NetXDuoEthIpInstance, &TCPSocket);

  /* never reached */
  for (;;) { tx_thread_sleep(NX_IP_PERIODIC_RATE); }
}

static UINT tcp_server_init(NX_IP *ip, NX_TCP_SOCKET *sock)
{
  UINT ret;

  /* Create TCP socket */
  ret = nx_tcp_socket_create(ip,
                             sock,
                             "TCP Server Socket",
                             NX_IP_NORMAL,
                             NX_FRAGMENT_OKAY,
                             NX_IP_TIME_TO_LIVE,
                             TCP_WINDOW_SIZE,
                             NX_NULL,
                             NX_NULL);
  if (ret != NX_SUCCESS)
  {
    printf("nx_tcp_socket_create failed: 0x%02X\r\n", (unsigned int)ret);
    return ret;
  }

  /* Listen once */
  ret = nx_tcp_server_socket_listen(ip,
                                    TCP_SERVER_PORT,
                                    sock,
                                    5,        /* backlog */
                                    NX_NULL);
  if (ret != NX_SUCCESS)
  {
    printf("nx_tcp_server_socket_listen failed: 0x%02X\r\n", (unsigned int)ret);
    return ret;
  }

  printf("TCP Server listening on port %u\r\n", (unsigned int)TCP_SERVER_PORT);
  return NX_SUCCESS;
}

static VOID tcp_server_run(NX_IP *ip, NX_TCP_SOCKET *sock)
{
  UINT   ret;
  ULONG  bytes_read;
  UCHAR  data_buffer[512];
  NX_PACKET *pkt;

  for (;;)
  {
    /* ACCEPT */
    ret = nx_tcp_server_socket_accept(sock, NX_WAIT_FOREVER);
    if (ret != NX_SUCCESS)
    {
      printf("nx_tcp_server_socket_accept failed: 0x%02X\r\n", (unsigned int)ret);

      /* Recover: make sure we're listening again */
      nx_tcp_server_socket_relisten(ip, TCP_SERVER_PORT, sock);
      continue;
    }

    printf("Client connected\r\n");

    /* RECEIVE LOOP (one-way: receive + UART print). If you want echo, add send back. */
    for (;;)
    {
      ret = nx_tcp_socket_receive(sock, &pkt, NX_IP_PERIODIC_RATE); /* 1s timeout */
      if (ret == NX_SUCCESS)
      {
        TX_MEMSET(data_buffer, 0, sizeof(data_buffer));

        nx_packet_data_retrieve(pkt, data_buffer, &bytes_read);

        if (bytes_read > 0U)
        {
          HAL_UART_Transmit(&huart3, (uint8_t*)data_buffer, (uint16_t)bytes_read, 0xFFFF);
        }

        nx_packet_release(pkt);
      }
      else if (ret == NX_NO_PACKET)
      {
        /* timeout: keep waiting */
        continue;
      }
      else
      {
        /* NX_NOT_CONNECTED / NX_CONNECTION_RESET / other errors */
        printf("Client disconnected or receive error: 0x%02X\r\n", (unsigned int)ret);
        break;
      }
    }

    /* CLEANUP (ST-style) */
    nx_tcp_socket_disconnect(sock, TCP_RECONNECT_TIMEOUT_TICKS);
    nx_tcp_server_socket_unaccept(sock);

    /* Re-listen without dropping the port */
    nx_tcp_server_socket_relisten(ip, TCP_SERVER_PORT, sock);

    printf("Waiting for new client...\r\n");
  }
}

/* USER CODE END 1 */
