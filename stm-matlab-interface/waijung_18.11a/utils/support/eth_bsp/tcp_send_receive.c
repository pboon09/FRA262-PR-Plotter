
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"

#include <string.h>

static struct tcp_pcb *tcp_server_pcb = NULL;
enum tcp_server_states
{
  TCP_SERVER_NONE = 0,
  TCP_SERVER_ACCEPTED,
  //TCP_SERVER_RECEIVED,  
	TCP_SERVER_CLOSING
};

#define TCP_RECEIVE_BUFFER_SIZE		512 /* Must be 2^N ;where N = 1, 2, 2, 4, ... */
#define TCP_TRANSMIT_BUFFER_SIZE	512 /* Must be 2^N ;where N = 1, 2, 2, 4, ... */

u8_t tcp_receive_buffer[TCP_RECEIVE_BUFFER_SIZE];
u8_t tcp_transmit_buffer[TCP_TRANSMIT_BUFFER_SIZE];

/* structure for maintaing connection infos to be passed as argument 
   to LwIP callbacks*/
struct tcp_server_struct
{
  u8_t state;             /* current connection state */
  struct tcp_pcb *pcb;    /* pointer on the current tcp_pcb */
  //struct pbuf *p;         /* pointer on the received/to be transmitted pbuf */
	
	/* Receive */
	u8_t *receive_buffer;
	u16_t receive_index;
	u16_t receive_count;
	/* Transmit */
	
};

struct tcp_server_struct tcp_working = 
{
	TCP_SERVER_NONE,
	NULL,
	NULL,
	0,
	0
};

#define TCP_RECEIVE_COUNT(a)	((a->count>a->index)?(a->count-a->index):(TCP_RECEIVE_BUFFER_SIZE-a->index+a->count))

static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
static void tcp_server_connection_close(struct tcp_pcb *tpcb, struct tcp_server_struct *es);
static void tcp_server_send(struct tcp_pcb *tpcb, struct tcp_server_struct *es);
static err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb);
static void tcp_server_error(void *arg, err_t err);
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);


/* Copy data from TCP buffer into local buffer */
void tcp_receive_copy(struct tcp_server_struct *local_buff,  struct pbuf *p) {
	struct pbuf *tmp;
	tmp = p;
	while(tmp) {
		int data_len;
		int buff_size;
		
		data_len = tmp->len;
		while(data_len) {		
			buff_size = TCP_RECEIVE_BUFFER_SIZE-local_buff->receive_count;
			if(buff_size >= data_len) {
				memcpy(local_buff->receive_buffer, p->payload, data_len);
				local_buff->receive_count += data_len;
				local_buff->receive_count &= (TCP_RECEIVE_BUFFER_SIZE-1);
				data_len = 0;
			}
			else {
				memcpy(local_buff->receive_buffer, p->payload, buff_size);
				local_buff->receive_count = 0;
				data_len -= buff_size;
			}
		}
		tmp = tmp->next;
	}
}

/**
  * @brief  Initializes the tcp echo server
  * @param  None
  * @retval None
  */
int tcp_server_init(void)
{
  /* create new tcp pcb */
  tcp_server_pcb = tcp_new();

  if (tcp_server_pcb != NULL)
  {
    err_t err;
    
    /* bind echo_pcb to port 7 (ECHO protocol) */
    err = tcp_bind(tcp_server_pcb, IP_ADDR_ANY, 7);
    
    if (err == ERR_OK)
    {
      /* start tcp listening for echo_pcb */
      tcp_server_pcb = tcp_listen(tcp_server_pcb);
      
      /* initialize LwIP tcp_accept callback function */
      tcp_accept(tcp_server_pcb, tcp_server_accept);
    }
    else 
    {
      return 1; // Can not bind pcb.
    }
  }
  else
  {
    return 2; //Can not create new pcb.
  }
	
	return 0;
}

/**
  * @brief  This function is the implementation of tcp_accept LwIP callback
  * @param  arg: not used
  * @param  newpcb: pointer on tcp_pcb struct for the newly created tcp connection
  * @param  err: not used 
  * @retval err_t: error status
  */
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
  err_t ret_err;
  struct tcp_server_struct *es;

  LWIP_UNUSED_ARG(arg);
  LWIP_UNUSED_ARG(err);

  /* set priority for the newly accepted tcp connection newpcb */
  tcp_setprio(newpcb, TCP_PRIO_MIN);

  /* allocate structure es to maintain tcp connection informations */
  es = (struct tcp_server_struct *)&tcp_working;//mem_malloc(sizeof(struct tcp_server_struct));
  if (es != NULL)
  {
    es->state = TCP_SERVER_ACCEPTED;
    es->pcb = newpcb;
    //es->p = NULL;
		es->receive_buffer = tcp_receive_buffer;
		es->receive_index = 0;
		es->receive_count = 0;
    
    /* pass newly allocated es structure as argument to newpcb */
    tcp_arg(newpcb, es);
    
    /* initialize lwip tcp_recv callback function for newpcb  */ 
    tcp_recv(newpcb, tcp_server_recv);
		
		// Krisada
		/* initialize LwIP tcp_sent callback function */
		tcp_sent(newpcb, tcp_server_sent);
    
    /* initialize lwip tcp_err callback function for newpcb  */
    tcp_err(newpcb, tcp_server_error);
    
    /* initialize lwip tcp_poll callback function for newpcb */
    tcp_poll(newpcb, tcp_server_poll, 1);
    
    ret_err = ERR_OK;
  }
  else
  {
    /* return memory error */
    ret_err = ERR_MEM;
  }
  return ret_err;  
}


/**
  * @brief  This function is the implementation for tcp_recv LwIP callback
  * @param  arg: pointer on a argument for the tcp_pcb connection
  * @param  tpcb: pointer on the tcp_pcb connection
  * @param  pbuf: pointer on the received pbuf
  * @param  err: error information regarding the reveived pbuf
  * @retval err_t: error code
  */
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  struct tcp_server_struct *es;
  err_t ret_err;

  LWIP_ASSERT("arg != NULL",arg != NULL);
  
  es = (struct tcp_server_struct *)arg;
  
  /* if we receive an empty tcp frame from client => close connection */
  if (p == NULL)
  {
    /* remote host closed connection */
    es->state = TCP_SERVER_CLOSING;
    //if(es->p == NULL)
    //{
    //   /* we're done sending, close connection */
    //   tcp_server_connection_close(tpcb, es);
    //}
    //else
    //{
    //  /* we're not done yet */
    //  /* acknowledge received packet */
    //  //tcp_sent(tpcb, tcp_server_sent);
    //  
    //  /* send remaining data*/
    //  //tcp_echoserver_send(tpcb, es);
    //}
    ret_err = ERR_OK;
  }
  /* else : a non empty frame was received from client but for some reason err != ERR_OK */
  else if(err != ERR_OK)
  {
    /* free received pbuf*/
    if (p != NULL)
    {
      //es->p = NULL;
      pbuf_free(p);
    }
    ret_err = err;
  }
  else if(es->state == TCP_SERVER_ACCEPTED)
  {
    /* first data chunk in p->payload */
    //es->state = TCP_SERVER_RECEIVED;
    
    /* store reference to incoming pbuf (chain) */
    //es->p = p; // ---------------------------------------!!!!!!!!!!!!!!!!!!
		tcp_receive_copy(es,  p);
    
    /* initialize LwIP tcp_sent callback function */
    //tcp_sent(tpcb, tcp_server_sent);
    
    /* send back the received data (echo) */
    //tcp_echoserver_send(tpcb, es);

    /* Acknowledge data reception */
    tcp_recved(tpcb, p->tot_len);		
		
		// free p here!!!
		pbuf_free(p);
    
    ret_err = ERR_OK;
  }
//   else if (es->state == TCP_SERVER_RECEIVED)
//   {
//     /* more data received from client and previous data has been already sent*/
// 		tcp_receive_copy(es,  p);
// 		
//     /* Acknowledge data reception */
//     tcp_recved(tpcb, p->tot_len);
// 		
// 		// free p here!!!
// 		pbuf_free(p);

//     //if(es->p == NULL)
//     //{
//     //  es->p = p;
// 		//
//     //  /* send back received data */
//     //  //tcp_echoserver_send(tpcb, es);
//     //}
//     //else
//     //{
//     //  struct pbuf *ptr;
// 		//
//     //  /* chain pbufs to the end of what we recv'ed previously  */
//     //  ptr = es->p;
//     //  pbuf_chain(ptr,p);
//     //}
//     ret_err = ERR_OK;
//   }
//   
  /* data received when connection already closed */
  else
  {
    /* Acknowledge data reception */
    tcp_recved(tpcb, p->tot_len);
    
    /* free pbuf and do nothing */
    //es->p = NULL;
    pbuf_free(p);
    ret_err = ERR_OK;
  }
  return ret_err;
}

/**
  * @brief  This function implements the tcp_err callback function (called
  *         when a fatal tcp_connection error occurs. 
  * @param  arg: pointer on argument parameter 
  * @param  err: not used
  * @retval None
  */
static void tcp_server_error(void *arg, err_t err)
{
  struct tcp_server_struct *es;

  LWIP_UNUSED_ARG(err);

  es = (struct tcp_server_struct *)arg;
  if (es != NULL)
  {
    /*  free es structure */
    mem_free(es);
  }
}

/**
  * @brief  This function implements the tcp_poll LwIP callback function
  * @param  arg: pointer on argument passed to callback
  * @param  tpcb: pointer on the tcp_pcb for the current tcp connection
  * @retval err_t: error code
  */
static err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb)
{
  err_t ret_err;
  struct tcp_server_struct *es;

  es = (struct tcp_server_struct *)arg;
  if (es != NULL)
  {
		
		tcp_server_send(tpcb, es);
    //if (es->p != NULL)
    //{
    //  /* there is a remaining pbuf (chain) , try to send data */
    //  tcp_server_send(tpcb, es);
    //}
    //else
    //{
      /* no remaining pbuf (chain)  */
      if(es->state == TCP_SERVER_CLOSING)
      {
        /*  close tcp connection */
        tcp_server_connection_close(tpcb, es);
      }
    //}
    ret_err = ERR_OK;
  }
  else
  {
    /* nothing to be done */
    tcp_abort(tpcb);
    ret_err = ERR_ABRT;
  }
  return ret_err;
}

/**
  * @brief  This function implements the tcp_sent LwIP callback (called when ACK
  *         is received from remote host for sent data) 
  * @param  None
  * @retval None
  */
static err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  struct tcp_server_struct *es;

  LWIP_UNUSED_ARG(len);

  es = (struct tcp_server_struct *)arg;
  
  //if(es->p != NULL)
  //{
  //  /* still got pbufs to send */
  //  tcp_server_send(tpcb, es);
  //}
  //else
  //{
    /* if no more data to send and client closed connection*/
    if(es->state == TCP_SERVER_CLOSING)
      tcp_server_connection_close(tpcb, es);
  //}
  return ERR_OK;
}


/**
  * @brief  This function is used to send data for tcp connection
  * @param  tpcb: pointer on the tcp_pcb connection
  * @param  es: pointer on echo_state structure
  * @retval None
  */
static unsigned int count_x = 0;
static void tcp_server_send(struct tcp_pcb *tpcb, struct tcp_server_struct *es)
{
  struct pbuf *ptr;
  err_t wr_err = ERR_OK;
	char buffer[32];
	
	sprintf(buffer, "%d\r\n", count_x++);
 
	if((es->state == TCP_SERVER_ACCEPTED) && (tpcb != NULL)) {
		wr_err = tcp_write(tpcb, buffer, strlen(buffer), 1);
	}
	
	
	
#if 0
	
  while ((wr_err == ERR_OK) &&
         (es->p != NULL) && 
         (es->p->len <= tcp_sndbuf(tpcb)))
  {
    
    /* get pointer on pbuf from es structure */
    ptr = es->p;

    /* enqueue data for transmission */
    wr_err = tcp_write(tpcb, ptr->payload, ptr->len, 1);
    
    if (wr_err == ERR_OK)
    {
      u16_t plen;

      plen = ptr->len;
     
      /* continue with next pbuf in chain (if any) */
      es->p = ptr->next;
      
      if(es->p != NULL)
      {
        /* increment reference count for es->p */
        pbuf_ref(es->p);
      }
      
      /* free pbuf: will free pbufs up to es->p (because es->p has a reference count > 0) */
      pbuf_free(ptr);

      /* Update tcp window size to be advertized : should be called when received
      data (with the amount plen) has been processed by the application layer */
      tcp_recved(tpcb, plen);
   }
   else if(wr_err == ERR_MEM)
   {
      /* we are low on memory, try later / harder, defer to poll */
     es->p = ptr;
   }
   else
   {
     /* other problem ?? */
   }
  }
#endif // 0	
	
}

void tcp_user(void)
{
	tcp_server_send(tcp_working.pcb, &tcp_working);
}

/**
  * @brief  This functions closes the tcp connection
  * @param  tcp_pcb: pointer on the tcp connection
  * @param  es: pointer on echo_state structure
  * @retval None
  */
static void tcp_server_connection_close(struct tcp_pcb *tpcb, struct tcp_server_struct *es)
{
  
  /* remove all callbacks */
  tcp_arg(tpcb, NULL);
  tcp_sent(tpcb, NULL);
  tcp_recv(tpcb, NULL);
  tcp_err(tpcb, NULL);
  tcp_poll(tpcb, NULL, 0);
  
  /* delete es structure */
  if (es != NULL)
  {
    mem_free(es);
  }  
  
  /* close tcp connection */
  tcp_close(tpcb);
}

