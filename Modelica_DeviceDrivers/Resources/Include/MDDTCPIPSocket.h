/** TCP/IP socket support (header-only library).
 *
 * @file
 * @author tbeu (Windows)
 * @author bernhard-thiele (Linux adaption)
 * @since 2015-04-18
 * @copyright see accompanying file LICENSE_Modelica_DeviceDrivers.txt
 *
 */

#ifndef MDDTCPIPSocket_H_
#define MDDTCPIPSocket_H_

#if !defined(ITI_COMP_SIM)

#include "ModelicaUtilities.h"
#include "MDDSerialPackager.h"

#if defined(_MSC_VER) || defined(__MINGW32__)

#include <ws2tcpip.h>
#include <stdio.h>
#include <stdlib.h>
#include "../src/include/CompatibilityDefs.h"

#pragma comment( lib, "Ws2_32.lib" )

typedef struct MDDTCPIPSocket_s MDDTCPIPSocket;

struct MDDTCPIPSocket_s {
    SOCKET SocketID;
};

DllExport void * MDD_TCPIPClient_Constructor(void) {
    MDDTCPIPSocket** tcpip = (MDDTCPIPSocket **)calloc(sizeof(MDDTCPIPSocket*), 1);
    if (tcpip) {
        *tcpip = (MDDTCPIPSocket *)calloc(sizeof(MDDTCPIPSocket), 1);
        if (*tcpip) {
            int rc; /* Error variable */
            WSADATA wsa;

            (*tcpip)->SocketID = INVALID_SOCKET;

            /* Initialize Winsock */
            rc = WSAStartup(MAKEWORD(2,2), &wsa);
            if (rc != NO_ERROR) {
                ModelicaFormatError("MDDTCPIPSocket.h: WSAStartup failed with error code: %d\n", rc);
            }
        }
    }

    return (void *) tcpip;
}

DllExport int MDD_TCPIPClient_Connect(void * p_tcpip, const char* ipaddress, int port) {
    MDDTCPIPSocket ** tcpip = (MDDTCPIPSocket **) p_tcpip;
    int ret = 0;
    if (tcpip && *tcpip) {
        if ((*tcpip)->SocketID == INVALID_SOCKET) {
            int rc; /* Error variable */
            struct addrinfo *result = NULL;
            struct addrinfo *ptr = NULL;
            struct addrinfo hints;
            char port_str[21];

            memset(&hints, 0, sizeof(hints));
            hints.ai_family = AF_UNSPEC;
            hints.ai_socktype = SOCK_STREAM;
            hints.ai_protocol = IPPROTO_TCP;

            /* Resolve the server address and port */
            _snprintf(port_str, 20, "%d", port);
            rc = getaddrinfo(ipaddress, port_str, &hints, &result);
            if (rc != NO_ERROR) {
                free(*tcpip);
                *tcpip = NULL;
                WSACleanup();
                ModelicaFormatError("MDDTCPIPSocket.h: getaddrinfo failed with error code: %d\n", rc);
            }

            /* Attempt to connect to an address until one succeeds */
            for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {

                /* Create a SOCKET for connecting to server */
                (*tcpip)->SocketID = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);
                if ((*tcpip)->SocketID == INVALID_SOCKET) {
                    free(*tcpip);
                    *tcpip = NULL;
                    rc = WSAGetLastError();
                    WSACleanup();
                    ModelicaFormatError("MDDTCPIPSocket.h: socket failed with error code: %d\n", rc);
                }

                /* Connect to server */
                rc = connect((*tcpip)->SocketID, ptr->ai_addr, (int)ptr->ai_addrlen);
                if (rc == SOCKET_ERROR) {
                    closesocket((*tcpip)->SocketID);
                    (*tcpip)->SocketID = INVALID_SOCKET;
                    continue;
                }
                break;
            }

            freeaddrinfo(result);

            if ((*tcpip)->SocketID == INVALID_SOCKET) {
                free(*tcpip);
                *tcpip = NULL;
                WSACleanup();
                ModelicaFormatError("MDDTCPIPSocket.h: Unable to connect to server!\n");
            }

            ret = 1;
        }
        else {
            ret = 1;
        }
    }
    return ret;
}

DllExport void MDD_TCPIPClient_Destructor(void * p_tcpip) {
    MDDTCPIPSocket ** tcpip = (MDDTCPIPSocket **) p_tcpip;
    if (tcpip) {
        if (*tcpip) {
            if ((*tcpip)->SocketID != INVALID_SOCKET) {
                shutdown((*tcpip)->SocketID, SD_BOTH);
                closesocket((*tcpip)->SocketID);
            }
            free(*tcpip);
            WSACleanup();
        }
        free(tcpip);
    }
}

DllExport int MDD_TCPIPClient_Send(void * p_tcpip, const char * data, int dataSize) {
    MDDTCPIPSocket ** tcpip = (MDDTCPIPSocket **) p_tcpip;
    int rc = 0;
    if (tcpip && *tcpip) {
        rc = send((*tcpip)->SocketID, data, dataSize, 0);
        if (rc == SOCKET_ERROR) {
            ModelicaFormatMessage("MDDTCPIPSocket.h: send failed with error code: %d\n", WSAGetLastError());
            rc = 1;
        }
    }
    return rc;
}

DllExport int MDD_TCPIPClient_SendP(void * p_tcpip, void* p_package, int dataSize) {
    return MDD_TCPIPClient_Send(p_tcpip, MDD_SerialPackagerGetData(p_package), dataSize);
}

DllExport const char * MDD_TCPIPClient_Read(void * p_tcpip, int recvbuflen) {
    MDDTCPIPSocket ** tcpip = (MDDTCPIPSocket **) p_tcpip;
    if (tcpip && *tcpip) {
        char* tcpBuf = ModelicaAllocateString(recvbuflen);
        if (tcpBuf) {
            int rc = recv((*tcpip)->SocketID, tcpBuf, recvbuflen, 0);
            if (rc == SOCKET_ERROR) {
                ModelicaFormatMessage("MDDTCPIPSocket.h: recv failed with error code: %d\n", WSAGetLastError());
            }
            return (const char*) tcpBuf;
        }
    }
    return "";
}

DllExport void MDD_TCPIPClient_ReadP(void * p_tcpip, void* p_package, int recvbuflen) {
    MDDTCPIPSocket ** tcpip = (MDDTCPIPSocket **) p_tcpip;
    if (tcpip && *tcpip) {
        char* tcpBuf = (char*) malloc(recvbuflen);
        if (tcpBuf) {
            int rc = recv((*tcpip)->SocketID, tcpBuf, recvbuflen, 0);
            if (rc == SOCKET_ERROR) {
                ModelicaFormatMessage("MDDTCPIPSocket.h: recv failed with error code: %d\n", WSAGetLastError());
            }
            rc = MDD_SerialPackagerSetDataWithErrorReturn(p_package, tcpBuf, rc);
            free(tcpBuf);
            if (rc) {
                ModelicaError("MDDTCPIPSocket.h: MDD_SerialPackagerSetData failed. Buffer overflow.\n");
            }
        }
    }
}

#elif defined(__linux__) || defined(__CYGWIN__)

#include <stdlib.h>
#include <string.h> /* memset(..) */
#include <errno.h>
#include <unistd.h> /* close */
#include <fcntl.h>  /* fcntl() */
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdio.h>
#include "../src/include/CompatibilityDefs.h"

typedef struct MDDTCPIPSocket_s MDDTCPIPSocket;

/** TCPIP socket object */
struct MDDTCPIPSocket_s {
    int sfd;  /**< socket file descriptor. */
};

/** Set socket to non-blocking mode.
 * @param p_tcpip pointer address to the tcpip socket data structure
 * @return 1 on success, 0 on failure
 */
int MDD_TCPIPClient_SetNonBlocking(void *p_tcpip) {
    MDDTCPIPSocket *tcpip = (MDDTCPIPSocket *)p_tcpip;
    int flags;

    if (!tcpip) return 0;

    flags = fcntl(tcpip->sfd, F_GETFL, 0);
    if (flags == -1) {
        ModelicaFormatError("MDDTCPIPSocket.h: fcntl(F_GETFL) failed (%s).\n", strerror(errno));
        return 0;
    }

    if (fcntl(tcpip->sfd, F_SETFL, flags | O_NONBLOCK) == -1) {
        ModelicaFormatError("MDDTCPIPSocket.h: fcntl(F_SETFL) failed (%s).\n", strerror(errno));
        return 0;
    }

    return 1;
}


/** Create a TCPIP socket.
 */
void * MDD_TCPIPClient_Constructor(void) {
    MDDTCPIPSocket *tcpip = (MDDTCPIPSocket *)malloc(sizeof(MDDTCPIPSocket));

    if (!tcpip) {
        ModelicaFormatError("MDDTCPIPSocket.h:%d: malloc() failed\n", __LINE__);
    }

    // Initialize socket descriptor to invalid value
    tcpip->sfd = -1;

    ModelicaFormatMessage("MDDTCPIPSocket.h: Created TCPIPClient structure\n");

    return (void *)tcpip;
}

/** Close socket and free memory.
 * @param p_tcpip pointer address to the tcpip socket data structure
 */
void MDD_TCPIPClient_Destructor(void *p_tcpip) {
    MDDTCPIPSocket *tcpip = (MDDTCPIPSocket *)p_tcpip;

    if (tcpip->sfd != -1) {
        if (close(tcpip->sfd) == -1) {
            ModelicaFormatError("MDDTCPIPSocket.h:%d: close() failed (%s)\n", __LINE__, strerror(errno));
        }
        ModelicaFormatMessage("Closed TCP/IP socket with socket handle %d\n", tcpip->sfd);
    }

    free(tcpip);
}


/** Connect client to server
 * @param p_tcpip pointer address to the tcpip socket data structure
 * @param ipaddress (Remote) IP address to connect to
 * @param port (Remote) port to connect to
 * @param useNonblockingMode If useNonblockingMode != 0, configure socket for non-blocking mode, otherwise blocking is enabled
 * @return returns 1
 */
int MDD_TCPIPClient_Connect(void *p_tcpip, const char *ipaddress, int port, int useNonblockingMode) {
    MDDTCPIPSocket *tcpip = (MDDTCPIPSocket *)p_tcpip;
    struct addrinfo hints;
    struct addrinfo *result, *rp;
    int s;
    char port_str[21];
    // socklen_t clilen;

    memset(&hints, 0, sizeof(struct addrinfo));
    hints.ai_family = AF_UNSPEC;    /* Allow IPv4 or IPv6 */
    hints.ai_socktype = SOCK_STREAM; /* TCP/IP socket */
    hints.ai_protocol = IPPROTO_TCP; /* TCP/IP protocol */

    /* Resolve the server address and port */
    ModelicaFormatMessage("Resolving server address  %s:%d ...\n", ipaddress, port);
    snprintf(port_str, 20, "%d", port);
    s = getaddrinfo(ipaddress, port_str, &hints, &result);
    if (s != 0) {
        ModelicaFormatError("MDDTCPIPSocket.h: getaddrinfo(..) failed (%s) \n", gai_strerror(s));
    }

    /* getaddrinfo() returns a list of address structures.
       Attempt to connect to an address until one succeeds */
    for (rp = result; rp != NULL; rp = rp->ai_next) {
        tcpip->sfd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if (tcpip->sfd == -1)
            continue;

        if (connect(tcpip->sfd, rp->ai_addr, rp->ai_addrlen) != -1) {
            if (useNonblockingMode) {
                if (!MDD_TCPIPClient_SetNonBlocking(tcpip)) {
                    ModelicaFormatMessage("Warning: Failed to set socket to non-blocking mode\n");
                }
            }
            break;  /* Success */
        }

        close(tcpip->sfd);
    }

    if (rp == NULL) {  /* No address succeeded */
        ModelicaFormatError("MDDTCPIPSocket.h:%d: Unable to connect to server.\n", __LINE__);
    } else {
      ModelicaFormatMessage("Connected to  %s:%d ...\n", ipaddress, port);
    }

    freeaddrinfo(result); /* No longer needed */

    return 1;
}

/** Send data via TCP/IP socket.
 *
 * Works for blocking and non-blocking socket.
 *
 * @param p_tcpip pointer address to the tcpip socket data structure
 * @param data pointer to data that should be sent
 * @param dataSize size of data to be sent in byte
 * @return returns 1
 */
int MDD_TCPIPClient_Send(void *p_tcpip, const char *data, int dataSize) {
    MDDTCPIPSocket *tcpip = (MDDTCPIPSocket *)p_tcpip;
    int amt, sent = 0;

    // ModelicaFormatMessage("MDDTCPIPSocket.h:%d MDD_TCPIPClient_Send BEGIN: %d\n", __LINE__, tcpip->sfd);

    /* Repeatedly call write until the entire buffer is sent. */
    while (sent < dataSize) {
        amt = write(tcpip->sfd, data + sent, dataSize - sent);

        if (amt > 0) {
            /* Update position by the number of bytes that were sent. */
            sent += amt;
        } else if (amt == 0) {
            /* Zero-byte writes are OK if they are caused by signals (EINTR).
               Otherwise they mean the socket has been closed. */
            if (errno == EINTR) {
                continue;
            }
            ModelicaFormatError("MDDTCPIPSocket.h:%d:, write() returned 0, connection may be closed.\n", __LINE__);
        } else {
            /* amt < 0 - check errno */
            if (errno == EINTR) {
                /* Interrupted by signal, just retry */
                continue;
            } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
                /* Use select() to wait until socket is ready for writing */
                fd_set writefds;
                struct timeval timeout;
                int select_result;

                FD_ZERO(&writefds);
                FD_SET(tcpip->sfd, &writefds);

                timeout.tv_sec = 5;   // 5 second timeout
                timeout.tv_usec = 0;

                select_result = select(tcpip->sfd + 1, NULL, &writefds, NULL, &timeout);

                if (select_result > 0) {
                    continue;  // Socket is ready, try writing again
                } else if (select_result == 0) {
                    ModelicaFormatError("MDDTCPIPSocket.h:%d: Write timeout after 5 seconds.\n", __LINE__);
                } else {
                    ModelicaFormatError("MDDTCPIPSocket.h:%d: select() failed (%s).\n", __LINE__, strerror(errno));
                }
            } else {
                ModelicaFormatError("MDDTCPIPSocket.h:%d: write(..) failed (%s).\n", __LINE__, strerror(errno));
                return 0;
            }
        }
    }

    // ModelicaFormatMessage("MDDTCPIPSocket.h:%d MDD_TCPIPClient_Send END: %d (sent %d bytes)\n", __LINE__, tcpip->sfd, sent);
    return 1;
}

/** Send data via TCP/IP socket.
 * @param p_tcpip pointer address to the tcpip socket data structure
 * @param p_package pointer to the SerialPackager
 * @param dataSize size of message to be sent in byte
 * @return returns 1
 */
int MDD_TCPIPClient_SendP(void *p_tcpip, void *p_package, int dataSize) {
    return MDD_TCPIPClient_Send(p_tcpip, MDD_SerialPackagerGetData(p_package), dataSize);
}

/** Read data from TCP/IP socket.
 *
 * @note No Modelica interface for this function, yet.
 *
 * @param p_tcpip pointer address to the tcpip socket data structure
 * @param recvbuflen length of message buffer
 * @return pointer to the message buffer
 */
const char * MDD_TCPIPClient_Read(void *p_tcpip, int recvbuflen) {
    MDDTCPIPSocket *tcpip = (MDDTCPIPSocket *)p_tcpip;
    ssize_t nread;
    char *tcpBuf = ModelicaAllocateString(recvbuflen);

    nread = read(tcpip->sfd, tcpBuf, recvbuflen);
    if (nread == -1) {
        ModelicaFormatError("MDDTCPIPSocket.h: read(..) failed (%s).\n",
                            strerror(errno));
    } else { /* Success */
        return (const char *)tcpBuf;
    }

    return "";
}

/** Read data from TCP/IP socket.
 *
 * @param p_tcpip pointer address to the tcpip socket data structure
 * @param p_package pointer to the SerialPackager
 * @param recvbuflen length of message buffer
 */
void MDD_TCPIPClient_ReadP_Blocking(void *p_tcpip, void *p_package, int recvbuflen) {
    MDDTCPIPSocket *tcpip = (MDDTCPIPSocket *)p_tcpip;
    ssize_t nread;
    int rc;
    char *tcpBuf = (char *)malloc(recvbuflen);

    nread = read(tcpip->sfd, tcpBuf, recvbuflen);
    if (nread == -1) {
        ModelicaFormatError("MDDTCPIPSocket.h: read(..) failed (%s).\n",
                            strerror(errno));
    }
    rc = MDD_SerialPackagerSetDataWithErrorReturn(p_package, tcpBuf, nread);
    free(tcpBuf);
    if (rc) {
        ModelicaError("MDDTCPIPSocket.h: MDD_SerialPackagerSetData failed. Buffer overflow.\n");
    }
}

/** Non-blocking read data from TCP/IP socket.
 * @param p_tcpip pointer address to the tcpip socket data structure
 * @param p_package pointer to the SerialPackager
 * @param recvbuflen length of message buffer
 */
void MDD_TCPIPClient_ReadP(void *p_tcpip, void *p_package, int recvbuflen) {
    MDDTCPIPSocket *tcpip = (MDDTCPIPSocket *)p_tcpip;
    ssize_t nread;
    int rc;
    char *tcpBuf;

    tcpBuf = (char *)malloc(recvbuflen);
    if (!tcpBuf) {
        ModelicaFormatError("MDDTCPIPSocket.h: malloc() failed.\n");
    }

    // ModelicaFormatMessage("MDDTCPIPSocketServer.h:%d: MDD_TCPIPServer_ReadP END\n", __LINE__);
    nread = read(tcpip->sfd, tcpBuf, recvbuflen);

    if (nread == -1) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // No data available right now - this is normal for non-blocking
            free(tcpBuf);
        } else {
            // Actual error
            free(tcpBuf);
            ModelicaFormatError("MDDTCPIPSocket.h:%d: read(..) failed (%s).\n", __LINE__, strerror(errno));
        }
    } else if (nread == 0) {
        // Connection closed by peer
        free(tcpBuf);
    } else {
        // Successfully read data
        rc = MDD_SerialPackagerSetDataWithErrorReturn(p_package, tcpBuf, nread);
        free(tcpBuf);

        if (rc) {
            ModelicaFormatError("MDDTCPIPSocket.h:%d: MDD_SerialPackagerSetData failed. Buffer overflow.\n", __LINE__);
        }
    }
    //ModelicaFormatMessage("MDDTCPIPSocket.h:%d: MDD_TCPIPClient_ReadP END: %d\n", __LINE__, tcpip->sfd);
}

/** Non-blocking read data from TCP/IP socket.
 * @param p_tcpip pointer address to the tcpip socket data structure
 * @param p_package pointer to the SerialPackager
 * @param recvbuflen length of message buffer
 * @param bytes_read pointer to store actual bytes read (can be NULL)
 * @return 1 on success (data read), 0 if no data available, -1 on error
 */
int MDD_TCPIPClient_ReadP_NonBlocking(void *p_tcpip, void *p_package, int recvbuflen, int *bytes_read) {
    MDDTCPIPSocket *tcpip = (MDDTCPIPSocket *)p_tcpip;
    ssize_t nread;
    int rc;
    char *tcpBuf;

    if (bytes_read) *bytes_read = 0;

    if (!tcpip || !p_package) return -1;

    tcpBuf = (char *)malloc(recvbuflen);
    if (!tcpBuf) {
        ModelicaFormatError("MDDTCPIPSocket.h: malloc() failed.\n");
        return -1;
    }

    nread = read(tcpip->sfd, tcpBuf, recvbuflen);

    if (nread == -1) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // No data available right now - this is normal for non-blocking
            free(tcpBuf);
            return 0;
        } else {
            // Actual error
            ModelicaFormatError("MDDTCPIPSocket.h: read(..) failed (%s).\n",
                                strerror(errno));
            free(tcpBuf);
            return -1;
        }
    } else if (nread == 0) {
        // Connection closed by peer
        free(tcpBuf);
        return -1;
    } else {
        // Successfully read data
        if (bytes_read) *bytes_read = (int)nread;

        rc = MDD_SerialPackagerSetDataWithErrorReturn(p_package, tcpBuf, nread);
        free(tcpBuf);

        if (rc) {
            ModelicaError("MDDTCPIPSocket.h: MDD_SerialPackagerSetData failed. Buffer overflow.\n");
            return -1;
        }

        return 1;
    }
}

#else

#error "Modelica_DeviceDrivers: No support of TCP/IP Socket for your platform"

#endif /* defined(_MSC_VER) */

#endif /* !defined(ITI_COMP_SIM) */

#endif /* MDDTCPIPSocket_H_ */
